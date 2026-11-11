param([string]$PythonExecutable = 'python', [string]$RuntimePath = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Path build/tests -Force | Out-Null
    $roots = @('Inc', 'Drivers', 'Middlewares', 'bsp', 'modules', 'application')
    $includes = @($roots | ForEach-Object {
        '-I' + $_
        Get-ChildItem -LiteralPath $_ -Directory -Recurse | ForEach-Object { '-I' + $_.FullName }
    })
    $flags = @('-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-std=gnu11', '-O1',
        '-ffunction-sections', '-fdata-sections', '-DUSE_HAL_DRIVER', '-DSTM32F407xx',
        '-DARM_MATH_CM4', '-DDISABLE_LOG_SYSTEM=1')
    $sources = @('tests/integration.c', 'tests/startup.s', 'application/cmd/robot_control.c', 'application/cmd/vision_command.c', 'modules/algorithm/controller.c',
        'modules/message_center/message_center.c', 'modules/daemon/daemon.c',
        'modules/motor/DJImotor/dji_motor.c', 'modules/can_comm/can_comm.c',
        'modules/algorithm/crc8.c')
    $previousPath = $env:PYTHONPATH
    try {
        if ($RuntimePath) { $env:PYTHONPATH = $RuntimePath }
        foreach ($sourceMode in 1) {
            $elf = "build/tests/integration-$sourceMode.elf"
            & arm-none-eabi-gcc @flags "-DSWERVE_COMMAND_SOURCE=$sourceMode" @includes @sources `
                -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
                '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $elf
            if ($LASTEXITCODE) { throw "Test compilation failed for command source $sourceMode" }
            & $PythonExecutable tests/emulate.py $elf
            if ($LASTEXITCODE) { throw "Integration tests failed for command source $sourceMode" }
        }
        $benchElf = 'build/tests/bench.elf'
        $benchSources = @('tests/bench.c', 'tests/startup.s', 'application/cmd/robot_control.c',
            'modules/message_center/message_center.c', 'modules/daemon/daemon.c',
            'modules/can_comm/can_comm.c', 'modules/algorithm/crc8.c')
        & arm-none-eabi-gcc @flags '-DCHASSIS_BENCH_TEST=1' @includes @benchSources `
            -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
            '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $benchElf
        if ($LASTEXITCODE) { throw 'Empty upper-board test compilation failed' }
        & $PythonExecutable tests/emulate.py $benchElf
        if ($LASTEXITCODE) { throw 'Empty upper-board tests failed' }
        $bspElf = 'build/tests/can-bsp.elf'
        & arm-none-eabi-gcc @flags @includes tests/can_bsp.c tests/startup.s `
            -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
            '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $bspElf
        if ($LASTEXITCODE) { throw 'CAN BSP test compilation failed' }
        & $PythonExecutable tests/emulate.py $bspElf
        if ($LASTEXITCODE) { throw 'CAN BSP tests failed' }
    } finally { $env:PYTHONPATH = $previousPath }
} finally { Pop-Location }

param(
    [switch]$SkipCBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot

function Invoke-Logged {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Label,
        [Parameter(Mandatory = $true)]
        [scriptblock]$Command
    )

    Write-Host "==> $Label"
    & $Command
}

function Get-PythonCommand {
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) {
        return @("py", "-3")
    }

    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($python) {
        return @("python")
    }

    throw "Python nao encontrado no PATH."
}

function Invoke-NativeCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Command,
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    & $Command @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Comando falhou com codigo ${LASTEXITCODE}: $Command $($Arguments -join ' ')"
    }
}

Push-Location $repoRoot
try {
    Invoke-Logged "Verificando arquivos obrigatorios" {
        $required = @(
            "README.md",
            ".gitignore",
            ".gitattributes",
            ".editorconfig",
            "Codigos_Python/AegisRover_BME688_DataPipeline/README.md",
            "Codigos_Python/AegisRover_BME688_BoschExporter/README.md",
            "Sistema_Navegacao_LiDAR/README.md"
        )

        foreach ($path in $required) {
            if (-not (Test-Path -LiteralPath $path)) {
                throw "Arquivo obrigatorio ausente: $path"
            }
        }
    }

    $pythonCommand = Get-PythonCommand
    $pythonExe = $pythonCommand[0]
    $pythonArgs = @()
    if ($pythonCommand.Count -gt 1) {
        $pythonArgs = $pythonCommand[1..($pythonCommand.Count - 1)]
    }

    Invoke-Logged "Compilando fontes Python" {
        $pythonFiles = Get-ChildItem -Recurse -File -Filter "*.py" |
            Where-Object {
                $_.FullName -notmatch "\\Softwares\\" -and
                $_.FullName -notmatch "\\__pycache__\\"
            } |
            ForEach-Object { $_.FullName }

        if (-not $pythonFiles) {
            throw "Nenhum arquivo Python encontrado."
        }

        & $pythonExe @pythonArgs -m py_compile @pythonFiles
    }

    $runningOnWindows = $IsWindows -or $env:OS -eq "Windows_NT"
    if ($runningOnWindows -and -not $SkipCBuild) {
        Write-Host "Windows detectado; build C Linux do NavSys_C ignorado. O GitHub Actions valida esta etapa em Ubuntu."
    }

    if (-not $SkipCBuild -and -not $runningOnWindows) {
        $gcc = Get-Command gcc -ErrorAction SilentlyContinue
        if ($gcc) {
            Invoke-Logged "Compilando alvos C do LiDAR" {
                $navRoot = Join-Path $repoRoot "Sistema_Navegacao_LiDAR/NavSys_C"
                $buildDir = Join-Path $navRoot "build"
                New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

                Push-Location $navRoot
                try {
                    $common = @("-std=c11", "-Wall", "-Wextra", "-O0", "-g")

                    Invoke-NativeCommand "gcc" ($common + @(
                        "-DHAL_PLATFORM=HAL_PLATFORM_LINUX",
                        "-Ihal",
                        "tests/test_hal_linux.c",
                        "hal/hal_linux.c",
                        "-lpthread",
                        "-o",
                        (Join-Path $buildDir "test_hal_linux")
                    ))

                    Invoke-NativeCommand "gcc" ($common + @(
                        "-DHAL_PLATFORM=HAL_PLATFORM_LINUX",
                        "-Ihal",
                        "-Inav_core",
                        "tests/test_parser.c",
                        "hal/hal_linux.c",
                        "nav_core/lidar_parser.c",
                        "-o",
                        (Join-Path $buildDir "test_parser")
                    ))

                    Invoke-NativeCommand "gcc" ($common + @(
                        "-Ihal",
                        "-Inav_core",
                        "tests/test_grid.c",
                        "hal/hal_linux.c",
                        "nav_core/lidar_parser.c",
                        "nav_core/scan_filter.c",
                        "nav_core/occupancy_grid.c",
                        "nav_core/grid_viz.c",
                        "-o",
                        (Join-Path $buildDir "test_grid")
                    ))

                    Invoke-NativeCommand "gcc" ($common + @(
                        "-Ihal",
                        "-Inav_core",
                        "tests/test_astar.c",
                        "hal/hal_linux.c",
                        "nav_core/lidar_parser.c",
                        "nav_core/scan_filter.c",
                        "nav_core/occupancy_grid.c",
                        "nav_core/grid_viz.c",
                        "nav_core/astar.c",
                        "-o",
                        (Join-Path $buildDir "test_astar")
                    ))

                    Invoke-NativeCommand "gcc" ($common + @(
                        "-Ihal",
                        "-Inav_core",
                        "app/linux/main.c",
                        "hal/hal_linux.c",
                        "nav_core/lidar_parser.c",
                        "nav_core/scan_filter.c",
                        "nav_core/occupancy_grid.c",
                        "nav_core/grid_viz.c",
                        "nav_core/astar.c",
                        "-lpthread",
                        "-o",
                        (Join-Path $buildDir "aegis_nav_mod")
                    ))
                }
                finally {
                    Pop-Location
                }
            }
        }
        else {
            Write-Host "gcc nao encontrado; build C ignorado."
        }
    }

    Write-Host "Validacao concluida sem erros."
}
finally {
    Pop-Location
}

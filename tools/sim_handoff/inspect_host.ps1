# Read-only: no WSL restart, software installation, or container changes.
$ErrorActionPreference = 'Continue'
Get-Date -Format o
Get-CimInstance Win32_OperatingSystem | Select-Object Caption, Version, BuildNumber, TotalVisibleMemorySize, FreePhysicalMemory
Get-CimInstance Win32_Processor | Select-Object Name, NumberOfCores, NumberOfLogicalProcessors
Get-CimInstance Win32_VideoController | Select-Object Name, DriverVersion
Get-PSDrive -PSProvider FileSystem | Select-Object Name, Used, Free
if (Get-Command wsl.exe -ErrorAction SilentlyContinue) {
    wsl.exe --version
    wsl.exe --list --verbose
}
if (Get-Command docker -ErrorAction SilentlyContinue) {
    docker version
    docker info --format '{{.OSType}} {{.Architecture}} CPUs={{.NCPU}} Memory={{.MemTotal}}'
    docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
    docker volume ls
}
if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
}

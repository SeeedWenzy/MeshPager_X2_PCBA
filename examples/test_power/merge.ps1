# Change to the directory where this script is located
Set-Location $PSScriptRoot

# Read the software version from the example's main.c
$mainC = Join-Path $PSScriptRoot "main\main.c"
if (-not (Test-Path $mainC)) {
    Write-Error "Source file not found: $mainC"
    exit 1
}

$versionMatch = Select-String -Path $mainC -Pattern '^\s*#define\s+APP_SW_VERSION\s+"([^"]+)"' | Select-Object -First 1
if ($null -eq $versionMatch -or [string]::IsNullOrWhiteSpace($versionMatch.Matches[0].Groups[1].Value)) {
    Write-Error "APP_SW_VERSION was not found in: $mainC"
    exit 1
}

$version = $versionMatch.Matches[0].Groups[1].Value
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$output = "meshpager_x2_merged_${version}_$timestamp.bin"

Write-Host "Start merging firmware..."

python -m esptool `
    --chip esp32s3 merge_bin `
    -o $output `
    0x0000 build/bootloader/bootloader.bin `
    0x8000 build/partition_table/partition-table.bin `
    0x10000 build/test_power.bin

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "================================="
    Write-Host "Merge Success!"
    Write-Host "Output: $output"
    Write-Host "================================="
}
else {
    Write-Host ""
    Write-Host "================================="
    Write-Host "Merge Failed!"
    Write-Host "================================="
    exit $LASTEXITCODE
}
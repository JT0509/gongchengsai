# Build the project with the source/inc list patched on the fly.
# Use this when EIDE's in-memory project model has lost APP/key, APP/nav,
# APP/steering or the new source files.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\fix_eide_build.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\fix_eide_build.ps1 --rebuild

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# 0) restore the fixed EIDE project file if EIDE overwrote it again
$fixedYml = Join-Path $root '.eide\eide.fixed.yml'
$liveYml = Join-Path $root '.eide\eide.yml'
if ((Test-Path -LiteralPath $fixedYml) -and (Test-Path -LiteralPath $liveYml)) {
    $liveText = Get-Content -LiteralPath $liveYml -Raw
    if (($liveText -notmatch 'APP/key') -or ($liveText -notmatch 'tcs3200')) {
        Copy-Item -LiteralPath $fixedYml -Destination $liveYml -Force
        Write-Host "eide.yml restored from eide.fixed.yml"
    }
}

# the params EIDE writes on each build are derived from its in-memory model,
# so they can be missing or stale: fall back to the last good copy
$params = Join-Path $root 'build\Target 1\builder.params'
$fixed = Join-Path $root 'build\Target 1\builder.fixed.params'
if (-not (Test-Path -LiteralPath $params)) {
    if (-not (Test-Path -LiteralPath $fixed)) {
        throw "builder.params not found: $params (run one EIDE build once to generate it)"
    }
    Copy-Item -LiteralPath $fixed -Destination $params -Force
    Write-Host "builder.params missing, seeded from builder.fixed.params"
}

$j = Get-Content -LiteralPath $params -Raw | ConvertFrom-Json

# 1) add missing include directories
$inc = @($j.incDirs)
foreach ($d in @('APP/key', 'APP/nav', 'APP/steering')) {
    if ($inc -notcontains $d) { $inc += $d }
}
$j.incDirs = $inc

# 2) drop stale L/R aggregator slots (those temp files were deleted 2026-10-24), add the real ones
$src = @($j.sourceList) | Where-Object { $_ -notmatch 'L_GREY_driver|R_GREY_driver' }

$need = @(
    'APP/color/tcs3200.c',
    'APP/key/key.c',
    'APP/nav/nav.c',
    'APP/sensor/S_GREY_driver.c',
    'APP/sensor/grey_i2c.c',
    'APP/steering/steering.c'
)

foreach ($f in $need) {
    if ($src -notcontains $f) { $src += $f }
}
$j.sourceList = $src

# 3) write the patched params next to the original
$j | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $fixed -Encoding UTF8

# 4) locate unify_builder from the installed EIDE extension
$builder = Get-ChildItem (Join-Path $env:USERPROFILE '.vscode\extensions') -Directory -Filter 'cl.eide-*' -ErrorAction SilentlyContinue |
Sort-Object Name -Descending |
ForEach-Object { Join-Path $_.FullName 'res\tools\win32\unify_builder\unify_builder.exe' } |
Where-Object { Test-Path -LiteralPath $_ } |
Select-Object -First 1

if (-not $builder) {
    throw 'unify_builder.exe not found under %USERPROFILE%\.vscode\extensions\cl.eide-*'
}

Write-Host "builder : $builder"
Write-Host "params  : $fixed"
Write-Host "sources : $($src.Count)"
Write-Host "incs    : $($inc.Count)"

& $builder -p $fixed @args
exit $LASTEXITCODE

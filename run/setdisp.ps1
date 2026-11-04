param(
    [switch]$List,
    [int]$Width = 1280,
    [int]$Height = 720,
    [int]$Hz = 60,
    [int]$DisplayIndex = 2
)

$cs = Get-Content -Raw (Join-Path $PSScriptRoot 'setdisp_types.cs')
Add-Type -TypeDefinition $cs -Language CSharp

function Cur($dev) {
    $dm = New-Object 'Disp+DEVMODE'
    $dm.dmSize = [System.Runtime.InteropServices.Marshal]::SizeOf($dm)
    if (-not [Disp]::EnumDisplaySettings($dev, -1, [ref]$dm)) { return $null }
    return $dm
}

$devs = @()
for ($n = 1; $n -le 8; $n++) {
    $name = '\\.\DISPLAY' + $n
    $dm = Cur $name
    if ($dm -eq $null) { continue }
    $devs += [pscustomobject]@{ Index = $n; Name = $name; W = $dm.dmPelsWidth; H = $dm.dmPelsHeight; Hz = $dm.dmDisplayFrequency }
    Write-Output ("[{0}] {1} current {2}x{3}@{4}" -f $n, $name, $dm.dmPelsWidth, $dm.dmPelsHeight, $dm.dmDisplayFrequency)
}

if ($List) {
    foreach ($d in $devs) {
        Write-Output ("--- modes of " + $d.Name + " ---")
        $dm = New-Object 'Disp+DEVMODE'
        $dm.dmSize = [System.Runtime.InteropServices.Marshal]::SizeOf($dm)
        $seen = @{}
        for ($i = 0; $i -lt 500; $i++) {
            if (-not [Disp]::EnumDisplaySettings($d.Name, $i, [ref]$dm)) { break }
            if ($dm.dmBitsPerPel -ne 32) { continue }
            $k = "{0}x{1}@{2}" -f $dm.dmPelsWidth, $dm.dmPelsHeight, $dm.dmDisplayFrequency
            if (-not $seen.ContainsKey($k)) {
                $seen[$k] = 1
                if (($dm.dmPelsWidth -eq 1280 -and $dm.dmPelsHeight -eq 720) -or ($dm.dmPelsWidth -eq 1920 -and $dm.dmPelsHeight -eq 1080)) {
                    Write-Output ("    " + $k)
                }
            }
        }
    }
    exit 0
}

$target = $devs | Where-Object { $_.Index -eq $DisplayIndex }
if (-not $target) { Write-Output "NO_SUCH_DISPLAY"; exit 2 }

$dmSet = New-Object 'Disp+DEVMODE'
$dmSet.dmSize = [System.Runtime.InteropServices.Marshal]::SizeOf($dmSet)
$found = -1
$seen = @{}
for ($i = 0; $i -lt 500; $i++) {
    if (-not [Disp]::EnumDisplaySettings($target.Name, $i, [ref]$dmSet)) { break }
    if ($dmSet.dmBitsPerPel -ne 32) { continue }
    if ($dmSet.dmPelsWidth -eq $Width -and $dmSet.dmPelsHeight -eq $Height -and $dmSet.dmDisplayFrequency -eq $Hz) { $found = $i; break }
    $k = "{0}x{1}@{2}" -f $dmSet.dmPelsWidth, $dmSet.dmPelsHeight, $dmSet.dmDisplayFrequency
    if (-not $seen.ContainsKey($k)) { $seen[$k] = 1 }
}
if ($found -lt 0) {
    Write-Output ("MODE_NOT_FOUND {0}x{1}@{2}; available(relevant):" -f $Width, $Height, $Hz)
    $seen.Keys | Where-Object { $_ -match '^(1280x720|1920x1080)' } | Sort-Object | ForEach-Object { Write-Output ("    " + $_) }
    exit 3
}
$dmSet = New-Object 'Disp+DEVMODE'
$dmSet.dmSize = [System.Runtime.InteropServices.Marshal]::SizeOf($dmSet)
[void][Disp]::EnumDisplaySettings($target.Name, $found, [ref]$dmSet)
$dmSet.dmFields = 0x00080000 -bor 0x00100000 -bor 0x00040000 -bor 0x00400000
$rc = [Disp]::ChangeDisplaySettingsEx($target.Name, [ref]$dmSet, [IntPtr]::Zero, 0, [IntPtr]::Zero)
Write-Output ("ChangeDisplaySettingsEx rc=" + $rc)
$now = Cur $target.Name
Write-Output ("now {0}x{1}@{2}" -f $now.dmPelsWidth, $now.dmPelsHeight, $now.dmDisplayFrequency)

$ErrorActionPreference = 'Stop'

$toolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $toolRoot '..\..')
$vendorDir = Join-Path $toolRoot '.vendor'
$firmwareSource = Join-Path $repoRoot 'MDK-ARM\glovesV1.0_VGT6\glovesV1_0_VGT6.hex'
$firmwareTarget = Join-Path $toolRoot 'resources\firmware\glovesV1_0_VGT6.hex'
$packSource = 'E:\Software\Apps\Keil\Package\Keil\STM32H5xx_DFP\2.2.0'
$packTarget = Join-Path $toolRoot 'resources\pack'

if (-not (Test-Path -LiteralPath $firmwareSource)) {
    throw "找不到 Keil 生成的 HEX：$firmwareSource"
}
if (-not (Test-Path -LiteralPath $packSource)) {
    throw "找不到 STM32H5 DFP：$packSource"
}

# 构建目录中的固件始终从 Keil 最新输出复制，避免把旧版本打进软件。
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $firmwareTarget) | Out-Null
Copy-Item -LiteralPath $firmwareSource -Destination $firmwareTarget -Force

$packFiles = @(
    'Keil.STM32H5xx_DFP.pdsc',
    'LICENSE',
    'CMSIS\Flash\STM32H5xx_1M_0800.FLM',
    'CMSIS\Flash\STM32H5xx_1M_0C00.FLM',
    'CMSIS\Debug\STM32H562xx_H563xx_H573xx.dbgconf',
    'CMSIS\SVD\STM32H563.svd'
)
foreach ($relativePath in $packFiles) {
    $source = Join-Path $packSource $relativePath
    $target = Join-Path $packTarget $relativePath
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
    Copy-Item -LiteralPath $source -Destination $target -Force
}

if (-not (Test-Path -LiteralPath (Join-Path $vendorDir 'pyocd'))) {
    python -m pip install --disable-pip-version-check --target $vendorDir 'pyocd==0.44.1'
}

$metadataPath = Join-Path $toolRoot 'resources\firmware.json'
$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$releaseVersion = ([string]($metadata.version)).Trim()
if ([string]::IsNullOrWhiteSpace($releaseVersion)) {
    throw "firmware.json 中的 version 不能为空"
}

# EXE 文件名与界面版本统一使用固件版本号。
$safeVersion = [regex]::Replace($releaseVersion, '[^0-9A-Za-z._-]', '_')
if ([string]::IsNullOrWhiteSpace($safeVersion)) {
    throw "firmware.json 中的 version 无法生成有效文件名"
}
$outputName = 'GloveDAPFlasher_{0}' -f $safeVersion
Write-Host ('Release version: [' + $releaseVersion + ']')
Write-Host ('Output filename: [' + $outputName + '.exe]')
$metadata.sha256 = (Get-FileHash -LiteralPath $firmwareTarget -Algorithm SHA256).Hash.ToLowerInvariant()
$metadata.build_time = (Get-Item -LiteralPath $firmwareSource).LastWriteTime.ToString('yyyy-MM-dd HH:mm')
$metadata | ConvertTo-Json | Set-Content -LiteralPath $metadataPath -Encoding UTF8

python (Join-Path $toolRoot 'generate_icon.py')
if ($LASTEXITCODE -ne 0) {
    throw "Windows 图标生成失败，退出码：$LASTEXITCODE"
}

Push-Location $toolRoot
try {
    python -m PyInstaller `
        --noconfirm `
        --clean `
        --onefile `
        --windowed `
        --name $outputName `
        --icon 'resources\app_icon.ico' `
        --paths $vendorDir `
        --collect-all pyocd `
        --collect-all cmsis_pack_manager `
        --collect-all libusb_package `
        --collect-submodules intelhex `
        --hidden-import pyocd.probe.cmsis_dap_probe `
        --add-data "$vendorDir\pyocd\debug\sequences\default_sequences.yaml;pyocd\debug\sequences" `
        --add-data "$vendorDir\pyocd\debug\sequences\sequences.lark;pyocd\debug\sequences" `
        --add-data "$vendorDir\pyocd\debug\svd\svd_data.zip;pyocd\debug\svd" `
        --add-binary "$vendorDir\cmsis_pack_manager\cmsis_pack_manager\cmsis_pack_manager.dll;cmsis_pack_manager\cmsis_pack_manager" `
        --add-data 'resources\firmware.json;resources' `
        --add-data 'resources\app_icon.svg;resources' `
        --add-data 'resources\firmware;resources\firmware' `
        --add-data 'resources\pack;resources\pack' `
        'app.py'
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller 构建失败，退出码：$LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

Write-Host "构建完成：$toolRoot\dist\$outputName.exe"

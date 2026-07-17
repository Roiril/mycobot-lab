# ロボットハンド操作 ランチャー APK ビルドスクリプト（PowerShell 5.1 互換 / && 不使用）
#
# パイプライン: keytool(初回のみ) -> aapt2 compile -> aapt2 link
#   -> javac -> d8 -> classes.dex 挿入 -> zipalign -> apksigner sign/verify
# 使用ツールはすべて Unity 同梱 Android SDK / OpenJDK の絶対パス。
#
# 実行: powershell -ExecutionPolicy Bypass -File build.ps1
# 生成物: dist\handteleop.apk（署名済み・検証済み）

$ErrorActionPreference = "Stop"

# ---- ツールチェーン絶対パス ----
$SDK = "C:\Program Files\Unity\Hub\Editor\2022.3.62f2\Editor\Data\PlaybackEngines\AndroidPlayer\SDK"
$JDK = "C:\Program Files\Unity\Hub\Editor\2022.3.62f2\Editor\Data\PlaybackEngines\AndroidPlayer\OpenJDK"
$BT  = Join-Path $SDK "build-tools\34.0.0"
$ANDROID_JAR = Join-Path $SDK "platforms\android-32\android.jar"   # targetSdk 32 でコンパイル

$JAVAC     = Join-Path $JDK "bin\javac.exe"
$KEYTOOL   = Join-Path $JDK "bin\keytool.exe"
$AAPT2     = Join-Path $BT  "aapt2.exe"
$D8        = Join-Path $BT  "d8.bat"
$ZIPALIGN  = Join-Path $BT  "zipalign.exe"
$APKSIGNER = Join-Path $BT  "apksigner.bat"

# .bat 系ツール（d8 / apksigner）が java を見つけられるよう JAVA_HOME / PATH を設定
$env:JAVA_HOME = $JDK
$env:PATH = (Join-Path $JDK "bin") + ";" + $env:PATH

# ---- ディレクトリ ----
$Here = $PSScriptRoot
$Res  = Join-Path $Here "res"
$Src  = Join-Path $Here "src"
$Manifest = Join-Path $Here "AndroidManifest.xml"
$Dist = Join-Path $Here "dist"
$Obj  = Join-Path $Dist "obj"
$Keystore = Join-Path $Here "debug.keystore"

$FinalApk = Join-Path $Dist "handteleop.apk"

function Assert-Ok($label) {
    if ($LASTEXITCODE -ne 0) {
        throw "[$label] failed with exit code $LASTEXITCODE"
    }
    Write-Host "[$label] OK"
}

function Assert-File($path, $label) {
    if (-not (Test-Path $path)) {
        throw "[$label] expected output not found: $path"
    }
}

# ---- 事前チェック ----
foreach ($t in @($JAVAC, $KEYTOOL, $AAPT2, $D8, $ZIPALIGN, $APKSIGNER, $ANDROID_JAR)) {
    if (-not (Test-Path $t)) { throw "toolchain path missing: $t" }
}

# ---- クリーン ----
if (Test-Path $Dist) { Remove-Item $Dist -Recurse -Force }
New-Item -ItemType Directory -Path $Dist -Force | Out-Null
New-Item -ItemType Directory -Path $Obj  -Force | Out-Null

Write-Host "=== 0. debug keystore ==="
if (-not (Test-Path $Keystore)) {
    Write-Host "generating debug keystore ..."
    & $KEYTOOL -genkeypair -v `
        -keystore $Keystore `
        -alias androiddebugkey `
        -keyalg RSA -keysize 2048 -validity 10000 `
        -storepass android -keypass android `
        -dname "CN=Android Debug,O=Android,C=US"
    Assert-Ok "keytool"
} else {
    Write-Host "debug keystore already exists, reuse."
}

Write-Host "=== 1. aapt2 compile (resources) ==="
$ResZip = Join-Path $Dist "res.zip"
& $AAPT2 compile --dir $Res -o $ResZip
Assert-Ok "aapt2 compile"
Assert-File $ResZip "aapt2 compile"

Write-Host "=== 2. aapt2 link (manifest + resources -> unsigned apk) ==="
$LinkedApk = Join-Path $Dist "linked.apk"
& $AAPT2 link `
    -o $LinkedApk `
    -I $ANDROID_JAR `
    --manifest $Manifest `
    --min-sdk-version 29 `
    --target-sdk-version 32 `
    -R $ResZip
Assert-Ok "aapt2 link"
Assert-File $LinkedApk "aapt2 link"

Write-Host "=== 3. javac (compile Java -> class) ==="
$JavaFiles = Get-ChildItem -Path $Src -Recurse -Filter *.java | ForEach-Object { $_.FullName }
& $JAVAC -source 8 -target 8 -encoding UTF-8 -cp $ANDROID_JAR -d $Obj $JavaFiles
Assert-Ok "javac"

Write-Host "=== 4. d8 (class -> classes.dex) ==="
$ClassFiles = Get-ChildItem -Path $Obj -Recurse -Filter *.class | ForEach-Object { $_.FullName }
& $D8 --release --min-api 29 --lib $ANDROID_JAR --output $Dist $ClassFiles
Assert-Ok "d8"
$Dex = Join-Path $Dist "classes.dex"
Assert-File $Dex "d8"

Write-Host "=== 5. classes.dex を apk へ挿入 ==="
$UnalignedApk = Join-Path $Dist "unaligned.apk"
Copy-Item $LinkedApk $UnalignedApk -Force
# python zipfile で STORED（無圧縮）追加。cwd を dist にしてエントリ名を classes.dex に固定
Push-Location $Dist
try {
    & python -c "import zipfile; z=zipfile.ZipFile('unaligned.apk','a',zipfile.ZIP_STORED); z.write('classes.dex','classes.dex'); z.close(); print('dex added')"
    Assert-Ok "insert dex"
} finally {
    Pop-Location
}

Write-Host "=== 6. zipalign ==="
$AlignedApk = Join-Path $Dist "aligned.apk"
& $ZIPALIGN -f -p 4 $UnalignedApk $AlignedApk
Assert-Ok "zipalign"
Assert-File $AlignedApk "zipalign"
# 検証
& $ZIPALIGN -c 4 $AlignedApk
Assert-Ok "zipalign -c"

Write-Host "=== 7. apksigner sign ==="
& $APKSIGNER sign `
    --ks $Keystore `
    --ks-key-alias androiddebugkey `
    --ks-pass pass:android `
    --key-pass pass:android `
    --out $FinalApk `
    $AlignedApk
Assert-Ok "apksigner sign"
Assert-File $FinalApk "apksigner sign"

Write-Host "=== 8. apksigner verify ==="
& $APKSIGNER verify --verbose $FinalApk
Assert-Ok "apksigner verify"

Write-Host ""
Write-Host "======================================================"
Write-Host " BUILD OK -> $FinalApk"
Write-Host "======================================================"
Write-Host "install:"
Write-Host "  adb -s 2G0YC1ZF7S06BW install -r `"$FinalApk`""
Write-Host "  adb -s 2G0YC1ZF890864 install -r `"$FinalApk`""

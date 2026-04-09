# FTC Build CLI Helper Script
# This script automates common build tasks for this project without requiring Android Studio.

$JAVA_HOME_17 = "C:\Program Files\Eclipse Adoptium\jdk-17.0.18.8-hotspot"
$ANDROID_SDK = "C:\Users\unwir\AppData\Local\Android\Sdk"

Write-Host "Checking Environment..." -ForegroundColor Cyan

# Verify Java 17
if (-not (Test-Path "$JAVA_HOME_17\bin\java.exe")) {
    Write-Error "Java 17 not found at $JAVA_HOME_17. Please install it with 'winget install EclipseAdoptium.Temurin.17.JDK'."
    exit 1
}

# Verify Android SDK
if (-not (Test-Path $ANDROID_SDK)) {
    Write-Error "Android SDK not found at $ANDROID_SDK. Update the script or local.properties."
    exit 1
}

Write-Host "Using Java 17 from: $JAVA_HOME_17" -ForegroundColor Green
Write-Host "Using Android SDK from: $ANDROID_SDK" -ForegroundColor Green

# Define available actions
$actions = @{
    "build"     = "assembleDebug"
    "clean"     = "clean"
    "install"   = "installDebug"
    "test"      = "test"
    "check"     = "lint"
}

if ($args.Count -eq 0) {
    Write-Host "`nUsage: .\build-cli.ps1 <command>" -ForegroundColor Yellow
    Write-Host "Available commands: build, clean, install, test, check" -ForegroundColor Yellow
    exit 0
}

$cmd = $actions[$args[0]]
if (-not $cmd) {
    Write-Error "Unknown command: $($args[0]). Use: build, clean, install, test, check"
    exit 1
}

Write-Host "`nExecuting: gradlew.bat $cmd`n" -ForegroundColor Cyan

# Execute Gradle with Java 17 override and custom build directory flag
$env:JAVA_HOME = $JAVA_HOME_17
.\gradlew.bat $cmd -PuseCustomBuildDir=true

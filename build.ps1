Remove-Item -Recurse -Force bld -ErrorAction SilentlyContinue
$options = @('--configuration', 'Release', '-p:DebugType=embedded')
dotnet publish ./FrenetSerretFramePredictor $options --framework net8.0 -o ./bld
Write-Host -NoNewLine 'Press any key to continue...'
$null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown')
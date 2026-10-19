param([string]$ProjectPath = '')
$ErrorActionPreference = 'Stop'
if (!$ProjectPath) { $ProjectPath = (Get-Content "$PSScriptRoot\project.json" -Raw | ConvertFrom-Json).project }
if (!(Test-Path -LiteralPath (Join-Path $ProjectPath 'jev_api\boat_client.py'))) { throw 'Pass -ProjectPath pointing to the cloned Unity project.' }
. "$PSScriptRoot\environment.ps1"
$health = Invoke-RestMethod http://127.0.0.1:8009/v1/models -TimeoutSec 5
$manifest = Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
if ($health.models[0].run -ne $manifest.kev_path) { throw 'Start the deployed Kev-0.8B service first.' }
$logFile = "$PSScriptRoot\logs\boat-$(Get-Date -Format 'yyyyMMdd-HHmmss').jsonl"
& "$PSScriptRoot\.venv\Scripts\python.exe" "$ProjectPath\jev_api\boat_client.py" --backend kev --endpoint http://127.0.0.1:8009/v1/systemone --model kev-latest --log $logFile
exit $LASTEXITCODE

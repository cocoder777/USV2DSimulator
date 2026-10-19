$ErrorActionPreference='Stop'
$manifest=Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
$health=Invoke-RestMethod http://127.0.0.1:8010/deployment -TimeoutSec 10
if ($health.revision -ne $manifest.revision -or $health.device -notlike 'cuda*' -or !$health.budget_guard) { throw 'Unexpected model/device/configuration.' }
$health | ConvertTo-Json

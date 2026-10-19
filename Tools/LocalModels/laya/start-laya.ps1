param([int]$StartupTimeoutSeconds=300)
$ErrorActionPreference='Stop'
. "$PSScriptRoot\environment.ps1"
$manifest=Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
if (!(Test-Path (Join-Path $manifest.path 'model.safetensors'))) { throw 'Pinned Laya weights are missing.' }
$probe=[Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback,8010)
try { $probe.Start() } catch { throw '127.0.0.1:8010 is occupied. Check the existing service first.' } finally { $probe.Stop() }
$stamp=Get-Date -Format 'yyyyMMdd-HHmmss'
$stdout="$PSScriptRoot\logs\service-$stamp.out.log"
$stderr="$PSScriptRoot\logs\service-$stamp.err.log"
$p=Start-Process -FilePath "$PSScriptRoot\.venv\Scripts\python.exe" -ArgumentList @(('"{0}"' -f "$PSScriptRoot\serve-local.py")) -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
@{pid=$p.Id;start_ticks=$p.StartTime.ToUniversalTime().Ticks.ToString();stdout=$stdout;stderr=$stderr} | ConvertTo-Json | Set-Content "$PSScriptRoot\service-process.json" -Encoding UTF8
$deadline=(Get-Date).AddSeconds($StartupTimeoutSeconds)
while ((Get-Date) -lt $deadline) {
 if ($p.HasExited) { throw "Laya exited ($($p.ExitCode)); see $stderr" }
 $health=$null
 try { $health=Invoke-RestMethod http://127.0.0.1:8010/deployment -TimeoutSec 2 } catch { Start-Sleep -Seconds 1; continue }
 if ($health.revision -ne $manifest.revision -or $health.device -notlike 'cuda*' -or !$health.budget_guard) { throw 'Unexpected model/device/budget guard. Inspect service logs.' }
 Write-Output "Laya ready: http://127.0.0.1:8010/v1/systemone ($($health.device), $($health.dtype)); revision=$($health.revision)"
 exit 0
}
throw "Laya startup exceeded $StartupTimeoutSeconds seconds; process retained for inspection. See $stderr"

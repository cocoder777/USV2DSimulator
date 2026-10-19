param([int]$StartupTimeoutSeconds = 300)
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\environment.ps1"
$manifest = Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
if (!(Test-Path -LiteralPath (Join-Path $manifest.kev_path 'head.pt'))) { throw 'Kev decision head is missing. Run model download first.' }
$probe = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 8009)
try { $probe.Start() } catch { throw '127.0.0.1:8009 is occupied. Stop the existing service first.' } finally { $probe.Stop() }
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$stdout = "$PSScriptRoot\logs\service-$stamp.out.log"
$stderr = "$PSScriptRoot\logs\service-$stamp.err.log"
$argsList = @('-m','kev.serve','--run',('"{0}"' -f $manifest.kev_path),'--fallback',('"{0}"' -f $manifest.kev_path),'--host','127.0.0.1','--port','8009','--device','cuda')
$process = Start-Process -FilePath "$PSScriptRoot\.venv\Scripts\python.exe" -ArgumentList $argsList -WorkingDirectory "$PSScriptRoot\source" -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
@{ pid=$process.Id; start_ticks=$process.StartTime.ToUniversalTime().Ticks.ToString(); stdout=$stdout; stderr=$stderr; run=$manifest.kev_path } | ConvertTo-Json | Set-Content "$PSScriptRoot\service-process.json" -Encoding UTF8
$deadline = (Get-Date).AddSeconds($StartupTimeoutSeconds)
while ((Get-Date) -lt $deadline) {
    if ($process.HasExited) { throw "Kev exited ($($process.ExitCode)). See $stderr" }
    $health = $null
    try { $health = Invoke-RestMethod http://127.0.0.1:8009/v1/models -TimeoutSec 2 }
    catch { Start-Sleep -Seconds 1; continue }
    $card = $health.models[0]
    if ($card.run -ne $manifest.kev_path -or $card.base -ne $manifest.base_id -or $card.device -notlike 'cuda*' -or $card.truncate_states) { throw 'Unexpected model/device/truncation identity' }
    Write-Output "Kev-0.8B ready: http://127.0.0.1:8009/v1/systemone ($($card.device), $($card.dtype))"
    exit 0
}
throw "Startup exceeded $StartupTimeoutSeconds seconds. Process retained for inspection. See $stderr"

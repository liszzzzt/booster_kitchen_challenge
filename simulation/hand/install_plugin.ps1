# Install / uninstall the revo2_hand physics extension into the Booster Studio virtual-robot container.
#   .\install_plugin.ps1                       # container papt7-t2, install, then docker restart
#   .\install_plugin.ps1 -Container xxx-t2
#   .\install_plugin.ps1 -Uninstall
#   .\install_plugin.ps1 -NoRestart
param(
    [string]$Container = "papt7-t2",
    [switch]$Uninstall,
    [switch]$NoRestart
)
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$plugin = Join-Path $here "plugin"

docker exec $Container rm -rf /tmp/revo2_plugin
docker cp "$plugin" "${Container}:/tmp/revo2_plugin"
# Windows checkouts may turn LF into CRLF; strip it before bash runs the script
docker exec $Container sed -i 's/\r$//' /tmp/revo2_plugin/install_in_container.sh
if ($Uninstall) {
    docker exec $Container bash /tmp/revo2_plugin/install_in_container.sh uninstall
} else {
    docker exec $Container bash /tmp/revo2_plugin/install_in_container.sh
}
if (-not $NoRestart) {
    Write-Host "restarting $Container ..."
    docker restart $Container
}

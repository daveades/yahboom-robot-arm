# Camera streamer for the DOFBOT vision pipeline. Runs on WINDOWS.
#
# usbipd cannot pass webcams into WSL (no isochronous transfer support),
# so the camera stays owned by Windows and ffmpeg serves it as an MJPEG
# stream over the network; scripts/camera.sh in the container reads it
# and publishes /image_raw.
#
# Usage (PowerShell):
#   .\stream_camera.ps1                         # stream "USB Camera"
#   .\stream_camera.ps1 -ListDevices            # find your camera's name
#   .\stream_camera.ps1 -Camera "HD Webcam"     # different camera
#   .\stream_camera.ps1 -Port 8091 -Fps 15
#
# ffmpeg exits whenever a client disconnects, so this loops forever and
# restarts it after 1 s. Ctrl-C stops the loop. Allow the Windows
# Firewall prompt on first run (the container must reach the port).

param(
    [string]$Camera = "USB Camera",
    [int]$Port = 8090,
    [string]$Size = "640x480",
    [int]$Fps = 30,
    [switch]$ListDevices
)

if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) {
    Write-Error "ffmpeg not found. Install it with:  winget install ffmpeg"
    exit 1
}

if ($ListDevices) {
    Write-Host "Video devices ffmpeg can see:" -ForegroundColor Cyan
    ffmpeg -hide_banner -list_devices true -f dshow -i dummy 2>&1 |
        Select-String '\((video|audio)\)'
    exit 0
}

$url = "http://0.0.0.0:$Port/cam.mjpg"
Write-Host "Streaming '$Camera' ($Size @ ${Fps}fps) on $url" -ForegroundColor Cyan
Write-Host "Container side: scripts/camera.sh   (Ctrl-C here to stop)" -ForegroundColor Cyan

while ($true) {
    ffmpeg -hide_banner -f dshow -video_size $Size -framerate $Fps `
        -i video="$Camera" -c:v mjpeg -q:v 6 -f mpjpeg -listen 1 $url
    Write-Host "ffmpeg exited (client disconnected?) - restarting in 1s" -ForegroundColor Yellow
    Start-Sleep 1
}

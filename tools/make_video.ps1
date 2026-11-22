# Builds the walkthrough video without any recording: Windows text-to-speech reads each slide's notes of
# dist/PathGuard_video_deck.pptx, PowerPoint attaches each clip to its slide, advances when it ends, and exports
# dist/PathGuard_video.mp4 (1080p). If the folder $voice (default: Desktop\PathGuard_voice) holds the presenter's own
# clips slide1..slide8 (.m4a/.mp3/.wav/.wma/.aac), those are used instead of text-to-speech.
# usage: powershell -ExecutionPolicy Bypass -File tools/make_video.ps1 [-voice <folder>]
param([string]$voice = (Join-Path ([Environment]::GetFolderPath("Desktop")) "PathGuard_voice"))
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$deck = Join-Path $root "dist\PathGuard_video_deck.pptx"
$work = Join-Path $root "dist\video_build"
$mp4 = Join-Path $root "dist\PathGuard_video.mp4"
New-Item -ItemType Directory -Force $work | Out-Null

# spoken forms of jargon
$say = @{ "IMU" = "I M U"; "ATE" = "A T E"; "EuRoC" = "Euroc"; "TUM-VI" = "T U M V I"; "ashdev7" = "ash dev seven";
          "4x" = "four times"; "evo" = "evo"; "PathGuard" = "Path Guard"; "GPS" = "G P S" }

Add-Type -AssemblyName System.Speech
$tts = New-Object System.Speech.Synthesis.SpeechSynthesizer
$tts.SelectVoice("Microsoft Hazel Desktop")
$tts.Rate = 0

$pp = New-Object -ComObject PowerPoint.Application
$pp.Visible = -1
$pres = $pp.Presentations.Open($deck, $false, $false, $true)   # with a window: CreateVideo needs it
$total = 0.0
foreach ($s in $pres.Slides) {
    $text = $s.NotesPage.Shapes.Placeholders.Item(2).TextFrame.TextRange.Text
    foreach ($k in $say.Keys) { $text = [regex]::Replace($text, "\b" + [regex]::Escape($k) + "\b", $say[$k]) }
    $own = Get-ChildItem -Path $voice -File -ErrorAction SilentlyContinue |
           Where-Object { $_.BaseName -ieq ("slide{0}" -f $s.SlideIndex) -and $_.Extension -match '^\.(m4a|mp3|wav|wma|aac)$' } |
           Select-Object -First 1
    if ($own) {
        $wav = $own.FullName
        $sh = New-Object -ComObject Shell.Application
        $fo = $sh.Namespace($own.DirectoryName); $len = $fo.GetDetailsOf($fo.ParseName($own.Name), 27)   # hh:mm:ss
        $sec = [TimeSpan]::Parse($len).TotalSeconds + 0.5   # Shell rounds to whole seconds
        $src = "your voice"
    } else {
        $wav = Join-Path $work ("slide{0}.wav" -f $s.SlideIndex)
        $tts.SetOutputToWaveFile($wav)
        $tts.Speak($text)
        $tts.SetOutputToNull()
        $bytes = (Get-Item $wav).Length
        $sec = ($bytes - 44) / (22050 * 2)   # default SAPI format: 22.05 kHz, 16-bit mono
        $src = "text-to-speech"
    }
    $audio = $s.Shapes.AddMediaObject2($wav, $false, $true, -100, -100, 10, 10)   # off-slide, embedded
    $audio.AnimationSettings.PlaySettings.PlayOnEntry = $true
    $audio.AnimationSettings.PlaySettings.HideWhileNotPlaying = $true
    $dur = [Math]::Round($sec + 0.5, 1)
    $s.SlideShowTransition.AdvanceOnTime = $true
    $s.SlideShowTransition.AdvanceTime = $dur
    $total += $dur
    "slide {0}: {1:N1} s ({2})" -f $s.SlideIndex, $dur, $src
}
"total {0:N1} s" -f $total
if ($total -gt 178) { "WARNING: longer than 3:00 - re-record the longest clips a little faster" }
$withAudio = Join-Path $work "PathGuard_video_narrated.pptx"
$pres.SaveAs($withAudio)
$pres.Close()   # export from a fresh open of the saved file (exporting the just-modified presentation fails)
$pres = $pp.Presentations.Open($withAudio, $true, $false, $true)
$tmp = Join-Path $env:TEMP "PathGuard_video.mp4"   # export outside OneDrive, then copy
if (Test-Path $tmp) { Remove-Item $tmp }
if (Test-Path $mp4) { Remove-Item $mp4 }
$pres.CreateVideo($tmp, $true, 5, 1080, 30, 90)
while ($pres.CreateVideoStatus -eq 1 -or $pres.CreateVideoStatus -eq 2) { Start-Sleep -Seconds 2 }
"video status {0} (3 = done)" -f $pres.CreateVideoStatus
if (Test-Path $tmp) { Copy-Item $tmp $mp4 }
$pres.Close()
$pp.Quit()
Get-Item $mp4 | Select-Object Name, Length

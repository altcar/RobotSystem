."C:\Users\c85-lau\.local\bin\claude.exe"
Set-Content "$env:USERPROFILE\.claude\settings.json" @'
{
  "env": {
    "ANTHROPIC_BASE_URL": "https://opencode.ai/zen/go",
    "ANTHROPIC_MODEL": "deepseek-v4.1-flash",
    "ANTHROPIC_DEFAULT_SONNET_MODEL": "deepseek-v4.1-flash[1M]",
    "ANTHROPIC_DEFAULT_OPUS_MODEL": "mimo-v2.6-flash[1M]",
    "ANTHROPIC_DEFAULT_HAIKU_MODEL": "deepseek-v4-flash[1M]",
    "ANTHROPIC_API_KEY": "",
    "ANTHROPIC_DEFAULT_SONNET_MODEL_NAME": "deepseek-v4.1-flash",
    "ANTHROPIC_DEFAULT_OPUS_MODEL_NAME": "mimo-v2.6-flash",
    "ANTHROPIC_DEFAULT_FABLE_MODEL": "muse-spark-1.3-contributor[1M]",
    "ANTHROPIC_DEFAULT_FABLE_MODEL_NAME": "muse-spark-1.3-contributor",
    "CLAUDE_CODE_AUTO_MODE_SERVER": "0"
  },
  "modelSettings": {
    "deepseek-v4.1-flash": {
      "effortLevel": "medium"
    }
  },
  "autoUpdatesChannel": "latest",
  "skipDangerousModePermissionPrompt": true,
  "hasCompletedOnboarding": true,
  "model": "sonnet"
}
'@

  pio run -t upload -t monitor



` to run pio
C:\Users\c85-lau\.platformio\penv\Scripts\pio.exe
The pip-installed one (what I've been calling) also works:


C:\Users\c85-lau\AppData\Roaming\Python\Python314\Scripts\pio.exe
Call it without the full path

Add the folder to your user PATH once, then open a new terminal:


[Environment]::SetEnvironmentVariable(
  "Path",
  [Environment]::GetEnvironmentVariable("Path", "User") + ";$env:USERPROFILE\.platformio\penv\Scripts",
  "User"
)

Two things to know:

This is a live snapshot. The session keeps appending as we talk, so re-run the copy at the end (or copy again before you switch computers) to capture the latest turns.

Restoring is a file drop, not an import. On the new machine the transcript goes under %USERPROFILE%\.claude\projects\<encoded-working-directory>\<same-uuid>.jsonl, then you resume by ID. The encoded folder name must match the project path Claude Code derives on that machine, so keeping the same folder path (same OneDrive path and username) makes it a straight copy.

I'm waiting on the docs check for the precise resume commands and version/auth caveats — I'll follow up with the exact restore steps and write them into a RESTORE.md next to the transcript.
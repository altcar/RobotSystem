."C:\Users\c85-lau\.local\bin\claude.exe"
Set-Content "$env:USERPROFILE\.claude\settings.json" @'
{
  "env": {
    "ANTHROPIC_BASE_URL": "https://opencode.ai/zen/go",
    "ANTHROPIC_MODEL": "deepseek-v4.1-flash",
    "ANTHROPIC_DEFAULT_SONNET_MODEL": "deepseek-v4.1-flash[1M]",
    "ANTHROPIC_DEFAULT_OPUS_MODEL": "mimo-v2.6-flash[1M]",
    "ANTHROPIC_DEFAULT_HAIKU_MODEL": "deepseek-v4-flash[1M]",
    "ANTHROPIC_API_KEY": "oc_sk_1fc09e96b797_KEhNViXOGiH4GKwDn0SacUTZzaGrOZwN",
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

# GitHub Actions Notification Setup

This document explains how to receive notifications about your STM32 build status.

## Default Notifications (Already Active)

### 1. GitHub Web/App Notifications
- Go to your GitHub repository
- Click "Watch" → "All Activity"
- You'll get notifications in GitHub's web interface and mobile app

### 2. Email Notifications from GitHub
- Go to GitHub Settings → Notifications
- Enable "Email" for "Actions"
- GitHub will email you when workflows fail

### 3. GitHub Issues (Configured)
- Failed builds on `master` branch automatically create GitHub issues
- Issues are labeled with `bug`, `build-failure`, `ci`, `urgent`
- Contains direct links to failed workflow runs

### 4. PR Comments (Configured)
- Failed builds on pull requests add comments to the PR
- Prevents merging broken code

## Additional Notification Options

### Option A: Email Notifications (via Action)

To set up direct email notifications:

1. **Add these secrets to your repository:**
   - Go to Repository → Settings → Secrets and variables → Actions
   - Add secrets:
     ```
     EMAIL_USERNAME: your-email@gmail.com
     EMAIL_PASSWORD: your-app-password
     ```

2. **For Gmail users:**
   - Enable 2-factor authentication
   - Generate an "App Password" for this purpose
   - Use the app password, not your regular password

3. **Update your workflow** to use the email action (already configured in `build.yml`)

### Option B: Slack Notifications

Add this step to your workflow:

```yaml
- name: Slack notification on failure
  if: failure()
  uses: 8398a7/action-slack@v3
  with:
    status: failure
    webhook_url: ${{ secrets.SLACK_WEBHOOK_URL }}
  env:
    SLACK_WEBHOOK_URL: ${{ secrets.SLACK_WEBHOOK_URL }}
```

Setup:
1. Create a Slack app and get webhook URL
2. Add `SLACK_WEBHOOK_URL` secret to your repository

### Option C: Discord Notifications

```yaml
- name: Discord notification on failure
  if: failure()
  uses: sarisia/actions-status-discord@v1
  with:
    webhook: ${{ secrets.DISCORD_WEBHOOK }}
    status: failure
    title: "STM32 Build Failed"
    description: "Build failed on commit ${{ github.sha }}"
```

Setup:
1. Create Discord webhook in your server
2. Add `DISCORD_WEBHOOK` secret to your repository

### Option D: Microsoft Teams

```yaml
- name: Teams notification on failure
  if: failure()
  uses: skitionek/notify-microsoft-teams@master
  with:
    webhook_url: ${{ secrets.TEAMS_WEBHOOK_URL }}
    needs: ${{ toJson(needs) }}
    job: ${{ toJson(job) }}
    steps: ${{ toJson(steps) }}
```

## Monitoring Build Status

### GitHub Actions Badge
Add this to your README.md to show build status:

```markdown
![Build Status](https://github.com/YOUR_USERNAME/stm32-recipes/workflows/STM32%20Build%20and%20Test/badge.svg)
```

### Check Build History
- Go to your repository → Actions tab
- View all workflow runs and their status
- Download build artifacts (ELF files) from successful runs

## Quick Setup Checklist

✅ **Immediate (No setup required):**
- [x] GitHub issue creation on failed master builds
- [x] PR comments on failed builds
- [x] Build status in Actions tab
- [x] Build artifacts upload

🔧 **Enable for better notifications:**
- [ ] GitHub email notifications (Settings → Notifications)
- [ ] Repository watch (Click "Watch" → "All Activity")
- [ ] Email action (add EMAIL_USERNAME/EMAIL_PASSWORD secrets)
- [ ] Slack/Discord/Teams (add webhook secrets)

## Recommendation

**For most users**: Enable GitHub email notifications and repository watching. This gives you:
- Email alerts for failed builds
- GitHub notifications for all activity
- Automatic issue creation for failed master builds
- PR comments for failed builds

This covers all the important notifications without requiring additional setup!

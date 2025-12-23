# How to Upload Your Hexapod Project to GitHub

Follow these step-by-step instructions to create a GitHub repository and upload your code.

## Prerequisites

- GitHub account (create one at https://github.com if you don't have one)
- Git installed on your computer
- The hexapod-robot-controller folder on your computer

---

## Step 1: Create GitHub Repository

1. **Go to GitHub:**
   - Visit https://github.com
   - Log in to your account

2. **Create New Repository:**
   - Click the **+** icon in top right
   - Select **"New repository"**

3. **Configure Repository:**
   - **Repository name:** `hexapod-robot-controller`
   - **Description:** "18-DOF hexapod robot with autonomous obstacle avoidance using Arduino and I2C communication"
   - **Visibility:** Public (recommended for resume/portfolio)
   - **DO NOT** initialize with README, .gitignore, or license (we already have these)
   - Click **"Create repository"**

4. **Copy Repository URL:**
   - You'll see a page with setup instructions
   - Copy the HTTPS URL (looks like: `https://github.com/YourUsername/hexapod-robot-controller.git`)

---

## Step 2: Install Git (if not already installed)

### Windows:
1. Download Git from: https://git-scm.com/download/win
2. Run installer with default options
3. Open "Git Bash" from Start menu

### Mac:
1. Open Terminal
2. Type: `git --version`
3. If not installed, follow prompts to install Xcode Command Line Tools

### Linux:
```bash
sudo apt-get install git  # Ubuntu/Debian
sudo yum install git       # CentOS/RHEL
```

---

## Step 3: Upload Code to GitHub

### Option A: Using Command Line (Recommended)

1. **Open Terminal/Git Bash**

2. **Navigate to your project folder:**
   ```bash
   cd path/to/hexapod-robot-controller
   ```
   
   Example:
   ```bash
   # Windows
   cd C:\Users\YourName\Documents\hexapod-robot-controller
   
   # Mac/Linux
   cd ~/Documents/hexapod-robot-controller
   ```

3. **Initialize Git repository:**
   ```bash
   git init
   ```

4. **Configure Git (if first time):**
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```

5. **Add all files:**
   ```bash
   git add .
   ```

6. **Create first commit:**
   ```bash
   git commit -m "Initial commit: Hexapod robot controller with obstacle avoidance"
   ```

7. **Connect to GitHub repository:**
   ```bash
   git remote add origin https://github.com/YourUsername/hexapod-robot-controller.git
   ```
   
   Replace `YourUsername` with your actual GitHub username!

8. **Push code to GitHub:**
   ```bash
   git branch -M main
   git push -u origin main
   ```

9. **Enter credentials if prompted:**
   - Username: Your GitHub username
   - Password: Your GitHub personal access token (NOT your password)
   
   **To create token:**
   - Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
   - Generate new token with `repo` permissions
   - Copy and save the token (you won't see it again!)

---

### Option B: Using GitHub Desktop (Easier for Beginners)

1. **Download GitHub Desktop:**
   - Visit: https://desktop.github.com/
   - Install and sign in with your GitHub account

2. **Add Repository:**
   - Click "File" → "Add local repository"
   - Browse to your `hexapod-robot-controller` folder
   - If prompted that it's not a Git repository, click "create a repository"

3. **Publish Repository:**
   - Click "Publish repository" button
   - Uncheck "Keep this code private" (for public repo)
   - Click "Publish repository"

4. **Done!** Your code is now on GitHub.

---

## Step 4: Verify Upload

1. **Visit your repository:**
   - Go to: `https://github.com/YourUsername/hexapod-robot-controller`

2. **You should see:**
   - ✅ README.md displaying with nice formatting
   - ✅ All folders: hexapod_controller, calibration, examples, docs
   - ✅ LICENSE file
   - ✅ .gitignore file

3. **Check README rendering:**
   - The README should display nicely with headers, code blocks, and tables
   - If it looks good, you're done!

---

## Step 5: Add to Your Resume

Now you can add the GitHub link to your resume:

**Project Description:**
```
Hexapod Robot Controller | Arduino, C++, PWM Control     December 2024
GitHub: github.com/YourUsername/hexapod-robot-controller

• Assembled and programmed 18-DOF Hexapod Robot using Arduino Uno 
  microcontroller to move in multi-legged coordinated locomotion based 
  on Inverse Kinematics principles
• [rest of your bullet points...]
```

---

## Common Issues & Solutions

### Issue: "fatal: not a git repository"
**Solution:**
```bash
git init
```

### Issue: "Permission denied (publickey)"
**Solution:** Use HTTPS URL instead of SSH:
```bash
git remote set-url origin https://github.com/YourUsername/hexapod-robot-controller.git
```

### Issue: "rejected because remote contains work"
**Solution:**
```bash
git pull origin main --allow-unrelated-histories
git push origin main
```

### Issue: "Support for password authentication was removed"
**Solution:** Create a Personal Access Token:
1. GitHub → Settings → Developer settings → Personal access tokens
2. Generate token with `repo` scope
3. Use token instead of password when prompted

---

## Making Updates Later

When you make changes to your code:

```bash
# 1. Navigate to project folder
cd path/to/hexapod-robot-controller

# 2. Check what changed
git status

# 3. Add changed files
git add .

# 4. Commit with message
git commit -m "Add obstacle avoidance improvements"

# 5. Push to GitHub
git push origin main
```

---

## Tips for a Professional Repository

### Add Photos/Videos
1. Create `docs/images/` folder
2. Add photos of your robot
3. Update README.md to include: `![Robot](docs/images/robot.jpg)`

### Add Demo Video
1. Upload video to YouTube
2. Add link to README: `[Watch Demo Video](https://youtube.com/...)`

### Pin Repository
1. Go to your GitHub profile
2. Click "Customize your pins"
3. Select this repository to feature it

### Add Topics/Tags
1. On your repository page, click the gear icon next to "About"
2. Add topics: `arduino`, `robotics`, `hexapod`, `embedded-systems`, `i2c`, `pwm`
3. This helps others find your project

---

## Next Steps

1. ✅ Upload code to GitHub
2. ✅ Add link to your resume
3. ✅ Take photos/videos of working robot
4. ✅ Update README with actual photos
5. ✅ Share link with Northrop Grumman application
6. ✅ Be ready to discuss in interview!

---

**Questions?** 
- GitHub Help: https://docs.github.com
- Git Tutorial: https://git-scm.com/book/en/v2

Good luck with your project! 🚀

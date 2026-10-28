# Get updates and share your work

The [course project](https://gitlab.nikhef.nl/graven/crazyflie-topical-lectures/)
contains the exercises, notes and code on its `course` branch. When the project
is public, you can clone and fetch it without an account. While it is private,
you need a Nikhef GitLab account with repository access. An account is also
needed to host a fork or open merge requests. The private teaching repository
is separate.

## Clone the course

Cloning downloads the notes and code into a new folder on your laptop, together
with the history Git uses to merge later updates. It does not install Python or
the course libraries; the installation guide covers that next.

Install Git using the [setup guide](software_setup.md#native-setup). On Windows,
open **PowerShell**; on macOS or Linux, open **Terminal**. After installing Git,
open a new terminal window and check:

```text
git --version
```

You should see a Git version number. If the command is not found, finish the Git
installation and reopen the terminal before continuing.

Choose where to keep the project. The commands below use your home folder
(`$HOME` works in both PowerShell and macOS/Linux shells); you can instead use
`cd` to enter another writable folder. Run each line separately:

```text
cd "$HOME"
git clone --branch course --single-branch https://gitlab.nikhef.nl/graven/crazyflie-topical-lectures.git
cd crazyflie-topical-lectures
git branch --show-current
```

The last command should print `course`. Its README links the installation guide,
daily programmes, exercises and supporting material. If Git asks for credentials,
the project still requires authenticated access. With an SSH key registered in
your Nikhef GitLab account, copy **Clone with SSH** from the course project's
**Code** menu and use that URL in place of the HTTPS URL in the clone command.

Check the downloaded files with `ls` on macOS/Linux or `Get-ChildItem` in
PowerShell. You should see `README.md`, `code/`, `scripts/`,
`requirements-hardware.txt` and `requirements-simulation.txt`.
This `crazyflie-topical-lectures` folder is the **project root** used by the installation
and exercise commands. Continue with the operating-system instructions in the
[installation guide](software_setup.md#native-setup), from this folder.

If cloning reports that `crazyflie-topical-lectures` already exists and is not empty,
keep that folder intact. If it is your existing course clone, open it and use
the [update instructions](#bring-course-updates-into-your-work) below. Otherwise,
choose a different parent folder and repeat the clone there. If cloning fails,
resolve that error before running the following `cd` or setup commands.

## Start a team branch

From your cloned project folder, create a branch for your experiments:

```text
git switch -c team-work
```

Use one shared repository/fork for the team's code,
with branches for experiments. Before your first commit, set your Git identity:

```text
git config user.name "Your Name"
git config user.email "your.email@example.org"
```

Review `git status` and commit specific files you changed, for example:

```text
git add code/day2/exercise_policies.py
git commit -m "Test our innovation threshold"
```

The supplied ignore rules keep local environments, radio settings and common
data/output folders out of commits. Review the staged diff with `git diff --cached`
before committing; keep personal details, credentials and raw flight recordings
outside a public fork. Store results elsewhere or in a private team location.
Check the commits you are about to push as well as the final files: deleting
sensitive content in a later commit leaves it available in earlier history.
If something private was already pushed, stop sharing it and contact the course
maintainers; another deletion commit cannot undo the disclosure.

## Bring course updates into your work

With your current changes committed on `team-work`, run:

```text
git fetch origin
git merge origin/course
```

Git combines the course updates with your commits. If both sides changed the
same lines, it reports a conflict: inspect the affected files with a teammate,
keep the intended result, run the relevant tests, then `git add` those files and
`git commit`. `git merge --abort` returns to the pre-merge state if you want to
defer the update. Commit your work first so it is recoverable before merging.

If requirements changed, rerun the native setup script from the setup guide
with the same Python version, then activate the environment again. After updates,
run the checks for the exercises you use. Changes to flight-related code or
configuration also need the team's hardware checks before another flight.

`RELEASE.json` records the course release and source-file hashes when published.
Your experiment edits naturally differ from that snapshot; record your own Git
commit alongside the release when comparing runs.

## Share improvements

On Nikhef GitLab, open the course project and choose **Fork** to create a copy in
your account or team namespace. Include the `course` branch. GitLab's
[fork instructions](https://gitlab.nikhef.nl/help/user/project/repository/forking_workflow.md)
describe the account and visibility options. Keep the initial clone as the update source:

```text
git remote rename origin upstream
git remote add origin https://gitlab.nikhef.nl/YOUR-NAMESPACE/crazyflie-topical-lectures.git
git push -u origin team-work
```

Replace `YOUR-NAMESPACE` with the fork's namespace. GitLab will ask you to
authenticate when pushing; follow your account's HTTPS token or SSH setup and
never place a token in a committed file. From this point, course updates use
`git fetch upstream` and `git merge upstream/course` instead of the commands
above; `origin` is your team's fork.

Open a merge request from your improvement branch to the original project's
**`course` branch**, with a short explanation and the tests you ran. Small bug
fixes, clearer instructions and reusable tooling are particularly useful. Share
full exercise solutions within your team rather than adding them to the public
starter by default. Course maintainers review proposed changes and incorporate
them into the release source before publishing, so later releases preserve them.

If you started from an offline ZIP, keep that folder and its work intact. Clone
the repository into a separate folder, copy across only your intended source or
notes changes, review the diff and commit them. Keep the existing environment
with the old folder and run setup in the clone; virtual environments can contain
absolute paths and should not be moved between folders.

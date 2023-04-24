# Contributing to `mujoco_ros_pkgs`

This is an open source project welcoming contributions. In this document we list a few requirements you should adhere to.

## Workflow
1. Browse Issues and Pull-Requests on Github to see if the bug or feature you want to fix/add has already been reported/requested.
If not, please create a new issue.

2. Fork the repository, if you havn't done it already. Then clone the forked project and add the upstream repository:
    ```sh
    git clone https://github.com/<your-username>/mujoco_ros_pkgs.git
    cd mujoco_ros_pkgs
    git remote add upstream https://github.com/ubi-agni/mujoco_ros_pkgs.git
    ```

3. Setup pre-commit:
    * Install pre-commit with `pip install pre-commit`
    * In the root directory of the repository run the following commands
    ```sh
    # sets up clang-tidy, clang-format and other utils on commit
    pre-commit install-hooks
    # sets up commit message check
    pre-commit install --hook-type commit-msg
    ```
    * Optionally set our commit template to be shown when you commit: `git config commit.template .housekeeping/git-commit-template.txt`

4. Develop your contribution

    * Make sure your fork is up to date with the noetic-devel branch of the upstream repository:
    ```sh
    git checkout noetic-devel
    git pull upstream noetic-devel
    ```
    * Create a branch for your contribution with a sensible name:
    ```sh
    git checkout -b add-featureX
    ```
    * As you code, commit your changes (following conventional commits style, the commit template will help you with that).
    pre-commit will automatically run clang-format and clang-tidy and make sure your commit message conforms with our template.

    * Write tests using Google Tests to test your code.

5. Propose changes via Pull Requests

   * When your contribution is ready and all tests are passing, make sure all your changes are pushed to your feature branch
   * Submit a pull request into `ubi-agni/mujoco_ros_pkgs:noetic-devel` with an informative title and a detailed description.
   * Please link all relevant issues to your PR.
   * The team will review your contribution and provide feedback. To incorporate changes recommended by the reviewers, commit edits to your branch, and push to the branch again (there is no need to re-create the pull request, it will automatically track modifications to your branch).
   * Once your pull request is approved by the reviewers, it will be merged into the main codebase.

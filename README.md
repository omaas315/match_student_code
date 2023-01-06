# match_student_code
This repository contains all code of the students working at the Institut of Assembly Technology of the Leibniz University Hannover.

## How to add your code
There are two methods for adding your code. The first is only using the Github-Webinterface. The second option is by cloning the repository and working on your computer. We advice to use the second method.

### First Method
1. Fork this repository by clicking on the `fork`-button: ![ForkButton](documentation/fork_button.png). Afterwards click `Create fork`.
2. Click on the `README.md`-file and then click on the edit-button: ![EditButton](documentation/edit_button.png) 
3. Add another line to the table at the bottom. (You can copy the line of the student above for the structure)
4. Column `Id`: Increase the previous Id by 1. Please memorize the `Id`, as it will be used in `step 10`. (always three numbers, f.e. `007` or `069`)
5. Column `Thesis title`: Insert the title of your thesis.
6. Column `Description`: Explain in one sentence what algorithm or code you developed in your thesis.
7. Type in a meaningful commit message and commit the changes.
8. Add a folder directly below `match_student_code` with a `README.md` file by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and select `Add new file`. 
9. Think of a name that summarizes your work/algorithm. This name is used in the next step.
10. Behind `match_student_code/` type `Id_NameOfStep9/README.md`. Insert the `Id` from `step 4` and the name you thought of from `step 9`.
11. Navigate into the created folder `Id_NameOfStep9`.
12. Add all packages and additional files into the folder by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and then `Upload files`. Drag and drop all folders or files from the Windows Explorer into the box on the web interface to upload.
13. Type in a meaningful commit message and commit the changes.
14. Add a `README.md`-file in your folder (see `step 10`) and use the structure of the example-README-file in `000_ExampleFolder`.
15. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
16. Type in a meaningful commit message and commit the changes.
17. To finish, please jump below the second method where you create a pull request to the main repository.

### Second Method
1. Fork this repository by clicking on the `fork`-button: ![ForkButton](documentation/fork_button.png). Afterwards click `Create fork`.
2. Clone the repository to your PC by executing the `git clone`-command with the forked repository.
3. Using any editor (f.e. VSCode), open the `README.md`-file in the `match_student_code` folder.
4. Add another line to the table at the bottom. (You can copy the line of the student above for the structure)
5. Column `Id`: Increase the previous Id by 1. Please memorize the `Id`, as it will be used in `step 10`. (always three numbers, f.e. `007` or `069`)
6. Column `Thesis title`: Insert the title of your thesis.
7. Column `Description`: Explain in one sentence what algorithm or code you developed in your thesis.
8. Type in a meaningful commit message and commit the changes.
9. Think of a name that summarizes your work/algorithm. This name is used in the next step.
10. Create a folder in `match_student_code` named `Id_NameOfStep9`. Insert the `Id` from `step 5` and the name you thought of from `step 9`.
11. Add all packages and additional files into the folder, f.e. by drag and drop.
12. Add a `README.md`-file in your folder (see `step 10`) and use the structure of the example-README-file in `000_ExampleFolder`.
13. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
14. Type in a meaningful commit message and commit the changes.
15. If not already done, push the changes to your forked repository.
16. To finish, see the next chapter where you create a pull request to the main repository.

### Pull Request
To merge your changes/added files into the official match-ROS repository, we need to create a Pull Request:
1. Navigate to the start page of your forked version of the `match_student_code` repository.
2. Check if the last commit message is by you. If not, you need to go into your forked version of the `match_student_code` repository.
3. Click on the `Pull Request`-button: ![PullRequestButton](documentation/pull_request_button.png) and then `New Pull Request` on the right side.
4. Compare two things with the following picture. Firstly, does the `base repository:` and `base:` match excactly. Secondly, is your forked version of the `match_student_code` repository listed behind `head repository` and is the branch selected that you commited your code to. 

![grafik](https://user-images.githubusercontent.com/50292612/211014212-b623642f-1ab7-4cd2-b9cb-03a260362e44.png)

5. Check if all changes are listed correctly. Did you add all necessary packages? Did you add all additional Matlab or Python scripts?
6. Create the pull request by clicking on the `Create Pull Request`-button.
7. Inform your supervisor about the newly created pull request.

## Content Overview
| Id | Thesis title | Description |
| --- | --- | --- |
| 000 | Example Folder | This folder contains templates and examples how it should look like |

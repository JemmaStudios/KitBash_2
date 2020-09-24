# KitBash_2
Native executable that merges an orphan manipulator OBJ8 file for X-plane onto an existing cockpit OBJ8.

Kitbash 2.0 allows you to create a generic object with a related manipulator file that can be used for ANY aircraft in X-plane 11.  The workflow is...
1) Create an object with animated switches, knobs, etc.  
2) Create an orphan manipulator object that has the ANIM_MANIP objects and controls defined that would overlay the base object. 
3) Place your base object in any aircraft in X-plane 11 using PlaneMaker. 
3a) use a text or code editor to fine tune all rotational angles and X,Y,Z offsets in the ACF file.
4) Run Kitbash 2.0 providing it with path/filename of your ACF file, filename of the new positioned object, path/filename of Orphaned OBJ file, path/filename of aircraft cockpit OBJ file.

Kitbash will find the positioned object, determine the yaw, pitch, roll, and X, Y, Z offsets of your positioned object.  Open the orphaned manipulator file, perform rotational and offset transformations for every vertex.  The open the aircraft cockpit OBJ file and append the calculated vertices as well as re-indexed IDX/IDX10's and ANIM_MANIP sections to the aircraft cockpit OBJ.

Known limitations:
You cannot kitbash an orphan manip file that has moving manipulators.  Yet...

WINDOWS USERS:
Just download the KITBASH.EXE file and run kitbash.exe from a command line prompt.

MAC USERS:
Download KITBASH and run ./kitbash from a command line.

LINUX USERS:
Download kitbash.cxx and do linux stuff to it.

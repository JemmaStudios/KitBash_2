/* kitbash.cxx

by Jeffory J. Beckers
MIT License

Assuming you've created a component OBJ and related OBJ with manipulator objects, KITBASH finds the component OBJ in your ACF
file and determines the X,Y,Z offsets and yaw, pitch, and roll angles.  it then applies rotational and offset transformation to
each vertex in your manipulator obj and appends it to the original aircraft cockpit obj.
*/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <iterator>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cmath>
#include <regex>
#include <cstdio>

using namespace std;

typedef chrono::high_resolution_clock Clock;
typedef chrono::duration<float> float_seconds;

// define constants
const string VERSION = "2.0.0a0017";
const bool kb_debug = false;

// define global variables
bool ow_switch = false;       // the global overwrite flag can be sent as a switch.

string string_to_lower (string tString) {
    // converts a string to all lower case.
    transform (tString.begin(), tString.end(), tString.begin(), ::tolower);
    return tString;
}

vector<string> split_string (string source, string delimeter) {
    // splits tString into a vector of substrings that were seperated by 'delimiter'.

    size_t tPos = 0;
    vector<string> sVector;
    string tString;
    while ((tPos = source.find(delimeter)) != string::npos) {
        tString = source.substr(0, tPos);
        sVector.push_back (tString);
        source.erase (0,tPos + delimeter.length());
    }
    sVector.push_back(source);
    return sVector;
}

string trim(const string s) { // removes whitespace characters from beginnig and end of string s
    const int l = (int)s.length();
    int a=0, b=l-1;
    char c;
    while(a<l && ((c=s.at(a))==' '||c=='\t'||c=='\n'||c=='\v'||c=='\f'||c=='\r'||c=='\0')) a++;
    while(b>a && ((c=s.at(b))==' '||c=='\t'||c=='\n'||c=='\v'||c=='\f'||c=='\r'||c=='\0')) b--;
    return s.substr(a, 1+b-a);
}

string strip_delimit_string (string source, string delimeter = " ") {
    // strips the string of all white space and replaces whitespace with delimiter
    
    // remove any whitespace from start and end of the string
    source = trim(source);

    // then replace any remaining white space with the delimeter
    source = regex_replace(source, regex ("\\s+"), delimeter);
    return source;
}

bool backup_cockpit_file(string xp_cockpit_fName) {
    // will make 999 attempts to save the original cockpit.obj to a .SAVExxx file where xxx is 001-999
    int i = 0;
    int is_done = 1;
    char ext_num[4];
    do {
        i++;
        if (i > 999) return false;
        snprintf(ext_num, 4, "%03d", i);
        string num_ext = ext_num;
        string new_cockpit_fName = xp_cockpit_fName + ".SAVE" + num_ext;
        is_done = rename(xp_cockpit_fName.c_str(), new_cockpit_fName.c_str());
    } while (is_done !=0);
    return true;
} 

class xp_vt {
    /*  defines the xp vertex class
        the vertex class has an x,y,z coordinate and can be transformed via rotation along the x,y,z axis
        and then offset via x,y,z positional offsets.
    */
    public:
        double  x, y, z;            // x, y, z coordinates of the VT
        double  nx, ny, nz;         // normal x, y, z coordinates of the VT
        double  u, v;               // x, y coordinate of uv map
        double  psi, theta, phi;    // psi, theta, phi rotational angles.
        double  off_x, off_y, off_z;// x, y, z offsets
    
    xp_vt (string vt_string) {
        /*  initialize the object by passing the string from an OBJ8 file in the format
            VT X Y Z NX NY NZ U V
            we'll break up the string and save the floats accordingly
        */

        // We'll first replace all white space characters with a single space
        vt_string = strip_delimit_string (vt_string);
        // then split the string into it's component parts
        vector<string> vt_parts = split_string (vt_string, " ");

        if (vt_parts[0].compare("VT") != 0) {  // It's not a VT string?  wtf, dude?
            x = 0;
            y = 0;
            z = 0;
            nx = 0;
            ny = 0;
            nz = 0;
            u = 0;
            v = 0;
        } else if (vt_parts.size() >= 9) { // that looks pretty good!
            x = stof (vt_parts[1]);     // x, y, z coords of VT
            y = stof (vt_parts[2]);
            z = stof (vt_parts[3]);
            nx = stof (vt_parts[4]);   // x,y,z vector of normal ray
            ny = stof (vt_parts[5]);
            nz = stof (vt_parts[6]);
            u = stof (vt_parts[7]);     // x,y uv map plot
            v = stof (vt_parts[8]);
        } else {                        // it went pear shaped.
            cout << "** ERROR! The following VT string was garbage: " << vt_string << endl;
        }
    }

    void set_rotation_axis (double tPsi, double tTheta, double tPhi) {
        // sets the psi (yaw), theta (pitch), and phi (roll) angles in degrees

        psi = tPsi;
        theta = tTheta;
        phi = tPhi;
    }

    void set_xyz_offsets (double xx, double yy, double zz) {
        off_x = xx;
        off_y = yy;
        off_z = zz;
    }

    map<char, double> rotation_transform () {
        // transforms x, y, z rotationally around psi, theta, phi angles
        // returns a vector with new [x1, y1, z1] coordinate array
        // https://en.wikipedia.org/wiki/Rotation_matrix for people that already understand it.
        // http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/ for dummies.

        #define PI 3.14159265
        double rad_phi = phi*PI/180;
        double rad_theta = theta*PI/180;
        double rad_psi = psi*PI/180;

        map<char, double> new_coords;
        int i = 0;
        int i_phi = 1;
        int i_psi = 2;
        int i_theta = 3;
        new_coords['x'] = x;
        new_coords['y'] = y;
        new_coords['z'] = z;

        // we rotate along the z/phi/roll axis
        double x_phi, y_phi, z_phi;

        x_phi = (cos(rad_phi) * new_coords['x']) + (sin(rad_phi) * new_coords['y']);
        y_phi = (-sin(rad_phi) * new_coords['x']) + (cos(rad_phi) * new_coords['y']);
        z_phi = new_coords['z'];

        new_coords['x'] = x_phi;
        new_coords['y'] = y_phi;
        new_coords['z'] = z_phi;

        // we rotate around the x/theta/pitch axis
        double x_theta, y_theta, z_theta;

        x_theta = new_coords['x'];
        y_theta = (cos(rad_theta) * new_coords['y']) + (-sin(rad_theta) * new_coords['z']);
        z_theta = (sin(rad_theta) * new_coords['y']) + (cos(rad_theta) * new_coords['z']);

        new_coords['x'] = x_theta;
        new_coords['y'] = y_theta;
        new_coords['z'] = z_theta;

        // we rotate around the y/psi/yaw axis
        double x_psi, y_psi, z_psi;

        x_psi = (cos(rad_psi) * new_coords['x']) + (-sin(rad_psi) * new_coords['z']);
        y_psi = new_coords['y'];
        z_psi = (sin(rad_psi) * new_coords['x']) + (cos(rad_psi) * new_coords['z']);

        new_coords['x'] = x_psi;
        new_coords['y'] = y_psi;
        new_coords['z'] = z_psi;

        // we're committed!
        
        x = new_coords['x'];
        y = new_coords['y'];
        z = new_coords['z'];

        return new_coords;
    }

    map<char,double> offset_tranform() {
        // offsets the x, y, z coordinates by off_x, off_y, off_z
        map<char, double> new_coords;

        x += off_x;
        y += off_y;
        z += off_z;
        new_coords['x'] = x;
        new_coords['y'] = y;
        new_coords['z'] = z;

        return new_coords;
    }

    void transform(double tPsi, double tTheta, double tPhi, double xx, double yy, double zz) {
        // accepts 3 rotation axis, and 3 offset axis. sets the members and performs rotation and offset transformations.
        set_rotation_axis (tPsi, tTheta, tPhi);
        set_xyz_offsets (xx, yy, zz);
        if ((tPsi != 0) || (tTheta != 0) || (tPhi != 0)) rotation_transform();
        if ((xx != 0) || (yy != 0) || (zz != 0)) offset_tranform();
    }

    stringstream get_vt_string(string end_line = "") {
        // returns formatted VT string using current data
        stringstream tString;
        tString     << "VT\t" << fixed << setprecision (8) << x << "\t" << y << "\t" << z << "\t" << nx << "\t" << ny << "\t" << nz
                    << "\t" << u << "\t" << v << end_line;
        return tString;
    }
};

class xp_acf_file {
    /* Defines the xp_acf_file class.  
    The ACF file contains important information about an aircraft type and also how the attached miscellaneous objects
    are offset and rotated to be placed properly in the aircraft.
    */
        string  xp_acf_fName = "",          // path and name of acf_file.
                xp_pObj_fName = "",         // name of positioned object to look for. 
                pObj_i_String;              // obj part index within the acf file (P _obja/x/ where x = pObj_index)        
        ifstream xp_acf_file;               // file class created from xp_acf_fName
    public:
        double  pObj_offset_x,              // offsets of positioned object relative to aircraft origin
                pObj_offset_y,
                pObj_offset_z,
                pObj_rotation_psi,          // rotation of positioned object relative to aircraft origin
                pObj_rotation_theta,
                pObj_rotation_phi;

        int set_acf_fName (string tName) {
            // verifies the file can be opened, and if so saves the file name to xp_acf_fName and returns 1.
            // returns 0 if the file can't be opened.

            xp_acf_file.open(tName);

            if (xp_acf_file.is_open()) {//Let's verify we can open the acf_file
                xp_acf_file.close();
                xp_acf_fName = tName;
                return 1;
            } else {
                return 0;
            }
        }  

        int set_pObj_fName (string tName) {
            /*  parses xp_acf_fName for a positioned object named xp_pObj_fName, if found it will then
                look up the object offsets and rotations and store then accordingly, and return 1.
                if it can't find the object within the parse or can't find the offsets, it will return 0.
                if someone calls this function before setting xp_acf_fName, it'll crash gracefully
            */
            if (xp_acf_fName.compare ("") == 0) {
                cerr    << "** Error! Attempt to set a xp_pObj_fName before setting xp_acf_fName in an xp_acf_file object type."
                        << endl;
                return 0;
            } else {
                string tLine;
                int found_it = 0;
                // now we'll parse the xp_acf_file for the object
                xp_acf_file.open(xp_acf_fName);
                if (xp_acf_file.is_open()) {
                    size_t found_pObj_index;
                    while (getline(xp_acf_file, tLine)) {
                        tLine = string_to_lower (tLine);
                        found_pObj_index = tLine.find(string_to_lower(tName));
                        if (found_pObj_index!=string::npos) {
                            xp_pObj_fName = tName;
                            found_it = 1;

                            vector<string> line_parts = split_string (tLine, " ");
                            line_parts = split_string(line_parts[1], "/");

                            // save the string we need to search for the pertinent data about xp_pObj_fName
                            pObj_i_String = "p _obja/" + line_parts[1] + "/";
                        }

                        // once we find the correct position object, let's get the data we need
                        string pObj_search_string;
                        if (found_it == 1) {
                            // looking for phi (roll) rotation
                            pObj_search_string = pObj_i_String + "_v10_att_phi_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> phi_parts = split_string (tLine, " ");
                                pObj_rotation_phi = stof (phi_parts[2]);
                            }
                            // looking for psi (yaw) rotation
                            pObj_search_string = pObj_i_String + "_v10_att_psi_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> psi_parts = split_string (tLine, " ");
                                pObj_rotation_psi = stof (psi_parts[2]);
                            }
                            // looking for theta (pitch) rotation
                            pObj_search_string = pObj_i_String + "_v10_att_the_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> theta_parts = split_string (tLine, " ");
                                pObj_rotation_theta = stof (theta_parts[2]);
                            }
                            // looking for x offset
                            pObj_search_string = pObj_i_String + "_v10_att_x_acf_prt_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> x_parts = split_string (tLine, " ");
                                pObj_offset_x = stof (x_parts[2]) * .3048 ;
                            }
                            // looking for y offset
                            pObj_search_string = pObj_i_String + "_v10_att_y_acf_prt_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> y_parts = split_string (tLine, " ");
                                pObj_offset_y = stof (y_parts[2]) * .3048;
                            }
                            // looking for z offset
                            pObj_search_string = pObj_i_String + "_v10_att_z_acf_prt_ref";
                            found_pObj_index = tLine.find(pObj_search_string);
                            if (found_pObj_index!=string::npos) {
                                vector<string> z_parts = split_string (tLine, " ");
                                pObj_offset_z = stof (z_parts[2]) * .3048;
                            }
                        }
                    }
                }

                return found_it;
            }
        } 

};

class xp_manip_file {
    /*  The manipulator.obj is an X-Plane OBJ8 text file that contains the geometer and anim_manip information required to
        control the positioned object.
        This class handles all the processing of that file including transforming all the vertices and re-indexing as necessary
        in preparation of appending to a cockpit OBJ file.
    */
        string xp_manip_fName;              // the file name of the manipulator obj.
        ifstream xp_manip_file;             // pointer to the file class;
        int vt_count;                       // number of VTs in this file;
        int idx_count;                      // number of IDXs in this file;
        vector<string> xp_vt_lines;         // a vector of lines comprised of each vt line in the file.
        vector<string> xp_idx_lines;        // a vector of lines comprised of each IDX line in the file.
        vector<string> xp_anim_footer;      // pretty much all the lines after the IDX section which should be all the anim lines. 

    public:

        int set_manip_fName (string tName) {
            // verifies the file can be opened, and if so saves the file name to xp_manip_fName and returns 1.
            // returns 0 if the file can't be opened.

            xp_manip_file.open(tName);

            if (xp_manip_file.is_open()) {//Let's verify we can open the manip_file
                xp_manip_file.close();
                xp_manip_fName = tName;
                return 1;
            } else {
                return 0;
            }
        }

        void transform_vts (xp_acf_file* t_acf_file) {
            /*  opens and reads in each line of the manipulator OBJ and transforms each VT line using the
                rotation and offset data from acf_file.  While we are here we're also going to load the IDX line vector.
            */

            bool found_idx = false;
            string tLine;
            xp_manip_file.open(xp_manip_fName);
            if (xp_manip_file.is_open()) {
                size_t xp_tmp_index;
                while (getline (xp_manip_file, tLine)) {
               // let's grab a little informatoin
                    xp_tmp_index = tLine.find("POINT_COUNTS");        // look for the POINT_COUNTS line
                    if (xp_tmp_index!=string::npos) {
                        tLine = strip_delimit_string (tLine, " ");
                        vector<string> pc_parts = split_string (tLine, " ");
                        vt_count = stoi (pc_parts[1]);
                        idx_count = stoi (pc_parts[4]);
                    }
                    xp_tmp_index = tLine.find("VT");        // look for VT lines
                    if (xp_tmp_index!=string::npos) {
                        tLine = strip_delimit_string (tLine);
                        xp_vt tVt(tLine);
                        tVt.transform(  t_acf_file->pObj_rotation_psi, t_acf_file->pObj_rotation_theta, t_acf_file->pObj_rotation_phi,
                                        t_acf_file->pObj_offset_x, t_acf_file->pObj_offset_y, t_acf_file->pObj_offset_z);
                        xp_vt_lines.push_back (tVt.get_vt_string("\n").str());  // add line to xp_vt_lines stack.
                    }
                    xp_tmp_index = tLine.find("IDX");
                    if (xp_tmp_index!=string::npos) {
                        if (!found_idx) found_idx = true;
                        tLine += "\n";
                        xp_idx_lines.push_back(tLine);
                    } else if (found_idx) { // if we previously found IDX lines, but now there are not any, we must be in the ANIM section.
                        tLine += "\n";
                        xp_anim_footer.push_back(tLine);
                    }
                }
            }
        }

        int get_vt_count() {
            return vt_count;
        }

        int get_idx_count() {
            return idx_count;
        }

        vector<string> get_vt_lines(){
            // returns the xp_vt_lines stack
            return xp_vt_lines;
        }  

        vector<string> get_idx_lines() {
            return xp_idx_lines;
        }

        vector<string> get_anim_footer() {
            return xp_anim_footer;
        }
        
};

class xp_cockpit_file {
    /*  The cockpit OBJ file contains the geometry and anim_manip information to allow x-plane users to interact with
        switches, knobs, and controls for a specific aircraft.  There can only be one such file per aircraft, thus the
        need for this utility.

        This class opens and processes the cockpit OBJ, obtains the necessary information needed to modify the manipulator
        object and will append the modified manipulator to the cockpit object file.
    */
        string xp_cockpit_fName;            // path and name of cockpit file
        ifstream xp_cockpit_file;           // file pointer to cockpit object
        vector<string> xp_cockpit_lines;    // vector of text lines comprising the cockpit object file
        int been_here_before = 0;           // has this manipulator object been kitbashed before?
        int orig_vt_count;                  // original number of VTs in the cockpit object file
        int orig_tris_count;                 // original number of indices in the cockpit object file
        int new_vt_count;                   // number of new VTs added by manipulator object
        int new_tris_count;                  // number of new IDX's added by manipulator object
        int orig_vt_end_index = 0;          // index to the last line of the VT section
        int orig_idx_end_index = 0;         // index to the last line of the IDX section
        int vt_lines_count = 0;             // no of VT lines in this file
        int idx_lines_count = 0;            // no of IDX or IDX10 lines in this file.
        bool is_analyzed = false;           // has this object been analyzed?

    public:

        int set_cockpit_fName (string tName) {
            // verifies the file can be opened, and if so saves the file name to xp_cockpit_fName and returns 1.
            // returns 0 if the file can't be opened.

            xp_cockpit_file.open(tName);

            if (xp_cockpit_file.is_open()) {//Let's verify we can open the cockpit_file
                xp_cockpit_file.close();
                xp_cockpit_fName = tName;
                return 1;
            } else {
                return 0;
            }
        }

        bool analyze_xp_cockpit_file (string pObj_name) {
            /*  reads through the cockpit OBJ file and finds the last line numbers for the VT section and IDX section
                to be used later.  Also determines if this item has been kitbashed before.
            */

            string tLine;                               // temp string used to store the line read from the file
            string tString;                             // temp string used for searches or writing new lines to the stack
            int linenum = 0;                            // current line number.
            size_t xp_tmp_index;                        // temporary index size for searching through lines
            xp_cockpit_file.open(xp_cockpit_fName);     // open the cockpit file
            if (xp_cockpit_file.is_open()) {            
                while(getline(xp_cockpit_file, tLine)) {
                    linenum++;
                    tLine = trim(tLine);
                    xp_tmp_index = tLine.find("VT");
                    if (xp_tmp_index == 0) {
                        // found VT in the first two chars so we'll save this line number.
                        orig_vt_end_index = linenum;
                        orig_vt_count ++;
                    }
                    xp_tmp_index = tLine.find("IDX");
                    if (xp_tmp_index == 0) {
                        orig_idx_end_index = linenum;
                    }
                }
                xp_cockpit_file.close();
                is_analyzed = true;
            } else return false;
            return true;                                   //everything is normal
        }

        int read_xp_cockpit_file (string pObj_name, xp_manip_file* manip_file, bool ow_flag) {
            /*  reads the cockpit file and populates the xp_cockpit_lines stack.  it also looks for the POINT_COUNTS
                line and sets the orig_vt_count and orig_tris_count members.

                parameters:
                    pObj_name:  name of the positioned object we're working with
                    manip_file: pointer to the manipulator object file we are using to modify this cockpit object
                    ow_flag:    if false, this routine will stop processing if it finds a "KITBASH - pObj_name" already
                                existing (presumable to prompt to overwrite), if set to true and it finds the same section
                                it will overwrite the old revised VT/IDX/ANIM_MANIP sections with the new information.

                returns:    0 if successful
                            1 if cockpit file could not be opened
                            2 if cockpit file could not be renamed
                            3 if pObj_name is already found and ow_flag = false
                            4 if the file could not be analyzed.
            */
            new_vt_count = manip_file->get_vt_count();
            new_tris_count = manip_file->get_idx_count();
           
            string tLine;
            string tString;
            bool found_idx = false;
            bool found_vts = false;
            bool ignoring_vt_lines = false;
            bool ignoring_idx_lines = false;
            bool ignoring_anim_lines = false;
            int line_num = 0;
            if (!is_analyzed) {
                // if this file hasn't been analyzed, we need to analyze it.  If it fails, we'll return gracefully.
                if (!analyze_xp_cockpit_file (pObj_name)) return 4;
            }
            xp_cockpit_file.open(xp_cockpit_fName);
            if (xp_cockpit_file.is_open()) {
                // we need to reset the vector of lines if we needed to run this twice.
                xp_cockpit_lines.clear();
 
                size_t xp_tmp_index;
                while (getline (xp_cockpit_file, tLine)) {
                    if (ignoring_vt_lines) {
                        tString = "# KITBASH - " + pObj_name + " end VT section";
                        xp_tmp_index = tLine.find(tString);
                        if (xp_tmp_index!=string::npos) {
                            // we got to the end of the VT section
                            ignoring_vt_lines = false;
                        }
                        continue;
                    }
                    xp_tmp_index = tLine.find("POINT_COUNTS");        // look for the POINT_COUNTS line
                    if (xp_tmp_index!=string::npos) {
                        tLine = strip_delimit_string (tLine, " ");
                        vector<string> pc_parts = split_string (tLine, " ");
                        orig_vt_count = stoi (pc_parts[1]);
                        orig_tris_count = stoi (pc_parts[4]);
                        stringstream ts;
                        ts << "# KITBASH - " << pObj_name << " VTs: " << new_vt_count << " TRIs: " << new_tris_count << "\n";
                        tString = ts.str();
                        xp_cockpit_lines.push_back(tString);
                        line_num++;
                        // need to add one to the vt and idx index
                        orig_vt_end_index++;
                        orig_idx_end_index++;
                        ts.str("");
                        ts  << "POINT_COUNTS " << orig_vt_count + new_vt_count << " " << pc_parts[2] << " " << pc_parts[3]
                            << " " << orig_tris_count + new_tris_count;
                        tLine = ts.str();
                    }
                    if ((line_num == orig_vt_end_index + 1) && (!found_vts)) {
                        // we're one line after the end of the original VT section.
                        found_vts = true;
                        tString = "# KITBASH - " + pObj_name + " start VT section\n";
                        xp_cockpit_lines.push_back(tString);
                        line_num++;
                        orig_vt_end_index++;
                        // add manip obj VTs
                        vector<string> manip_vt_lines = manip_file->get_vt_lines();
                        for (string vtString: manip_vt_lines) {
                            xp_cockpit_lines.push_back(vtString);
                            line_num++;
                            orig_idx_end_index++;
                            orig_vt_end_index++;
                        } 
                        tString = "# KITBASH - " + pObj_name + " end VT section\n";
                        xp_cockpit_lines.push_back(tString);
                        line_num++;
                        orig_vt_end_index++;
                        orig_idx_end_index++;
                    }
                    if ((line_num == orig_idx_end_index + 1) && (!found_idx)) {
                        // we've found the line after the original IDX section.
                        found_idx = true;
                        tString = "# KITBASH - " + pObj_name + " start IDX section\n";
                        xp_cockpit_lines.push_back(tString);
                        line_num ++;
                        orig_idx_end_index++;
                        // re-index and add manip obj IDX/IDX10 lines
                        vector<string> manip_idx_lines = manip_file->get_idx_lines();
                        for (string idxString: manip_idx_lines) {
                            idxString = strip_delimit_string(idxString);
                            vector<string> idx_parts = split_string(idxString, " ");
                            stringstream ts;
                            int n;
                            ts  << idx_parts[0];
                            for (size_t j = 1; j != idx_parts.size(); j++) {
                                n = stoi(idx_parts[j]) + orig_vt_count;
                                ts << " " << n;
                            }
                            ts << "\n";
                            xp_cockpit_lines.push_back(ts.str());
                            line_num ++;
                            orig_idx_end_index++;
                        }
                        tString = "# KITBASH - " + pObj_name + " end IDX section\n";
                        xp_cockpit_lines.push_back(tString);
                        line_num++;
                        orig_idx_end_index++;
                    }
                    // everything else should be the anim section
                    tLine += "\n";
                    xp_cockpit_lines.push_back(tLine);
                    line_num++;
                }
                // now we are at the end of the file we can add our ANIM section
                tString = "# KITBASH - " + pObj_name + " start ANIM section\n";
                xp_cockpit_lines.push_back(tString);
                line_num++;
                vector<string> manip_anim_footer = manip_file->get_anim_footer();
                for (string animString: manip_anim_footer) {
                    // if any of the lines are TRIS, then we need to modify it to add the orig_tris_count
                    xp_tmp_index = animString.find("TRIS");
                    if (xp_tmp_index != string::npos) {
                        animString = strip_delimit_string(animString, " ");
                        vector<string> anim_parts = split_string (animString, " ");
                        int new_tris = stoi(anim_parts[1]) + orig_tris_count;
                        stringstream idx_s;
                        idx_s << anim_parts[0] << " " << new_tris << " " << anim_parts[2] << "\n";
                        animString = idx_s.str();
                    }
                    xp_cockpit_lines.push_back(animString);
                    line_num++;
                }
                tString = "# KITBASH - " + pObj_name + " end ANIM section\n";
                xp_cockpit_lines.push_back(tString);
                line_num++;
                tString = "# KITBASH 2.0 by Jemma Studios.  Donations are motivation. https://paypal.me/JemmaStudios\n";
                xp_cockpit_lines.push_back(tString);
                line_num++;
                xp_cockpit_file.close();
            } else return 1;
            // Let's make a backup.

            if (!backup_cockpit_file(xp_cockpit_fName))
                return 2;
            else {
                ofstream output_file;
                output_file.open (xp_cockpit_fName);
                for (string tString: xp_cockpit_lines) {
                    output_file << tString;
                }
                output_file.close();
            }    
            return 0;
        }
        
        int get_vt_count(int opt=0) {
            switch (opt) {
                case 1:
                    return new_vt_count;
                    break;
                case 2:
                    return orig_vt_count + new_vt_count;
                    break;
                default:
                    return orig_vt_count;
            };

        }

        int get_idx_count(int opt=0) {
            switch (opt) {
                case 1:
                    return new_tris_count;
                    break;
                case 2:
                    return orig_tris_count + new_tris_count;
                    break;
                default:
                    return orig_tris_count;
            };
        }

        int get_vt_lines_count() {

            return vt_lines_count;
        }

        int get_idx_lines_count() {

            return idx_lines_count;
        }

        int get_orig_vt_end_index() {

            return orig_vt_end_index;
        }

        int get_orig_idx_end_index() {

            return orig_idx_end_index;
        }

};

static void print_usage() {
    // Prints out usage syntax for kitbash.exe

    cerr   << "Usage: kitbash <switches> <options>\n"
                << "Switches:\n"
                << "\t-h\t\tShow this help message.\n"
                << "\t-o\t\tOverride all user prompts and go with what get's the job done.\n"
                << "Options: (* indicates required option)\n"
                << "\t* -a ACF_FILENAME\tSpecify ACF path and file name.\n"
                << "\t* -p OBJECT_FILENAME\tName of positioned OBJ object within ACF file.\n"
                << "\t* -m MANIP_FILENAME\tSpecify manipulator.obj path and file name related to OBJECT_FILENAME.\n"
                << "\t* -c COCKPIT_FILENAME\tSpecify cockpit.obj path and file name that you want MANIP_FILENAME appended to.\n"
                << endl;
}

int arg_handler (int argc, char* argv[], string &acf_fName, string &pObj_name, string &mObj_fName, string &cObj_fName) {
    // Handles command line arguments

    // Set up a map of required options to check for later. We'll set to 0 for now since we don't have any yet.
    map<string, int> optList;
    optList["-a"] = 0;
    optList["-p"] = 0;
    optList["-m"] = 0;
    optList["-c"] = 0;

    // if we didn't send any options we'll post a usage message
    if (argc <= 1) {
        print_usage();
        return 1;
    }
    // let's parse the switches and options
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        char *opta = &arg[0];
        if (opta[0] != '-') {//what the heck? it's not a switch!
            cerr << "**ERROR! " << arg << " is an invalid switch format!\n" << endl;
            print_usage();
            return 1;
        }
        char opt = tolower(opta[1]);
        switch (opt) {
            case 'h':
                print_usage();
                return 1;
                break;
            case 'o':
                ow_switch = true;
                break;
            case 'a':
                if (i + 1 < argc) {//if we're looking for an .acf file there better be another argument
                    string tName = argv[++i];
                    acf_fName = tName;
                    optList["-a"] = 1;   // we'll set the correct option flag
                } else {
                    cerr << "**ERROR! No ACF filename provided for the -a switch! **\n" << endl;
                    print_usage();
                    return 1;
                }
                break;
            case 'p':
                if (i + 1 < argc) {//now we look for the positioned OBJ name
                    string tName = argv[++i];
                    pObj_name = tName;
                    optList["-p"] = 1;
                } else {
                    cerr << "**ERROR! No filename provided for the -p switch! **\n" << endl;
                    print_usage();
                    return 1;
                }
                break;
            case 'm':
                if (i + 1 < argc) { // look for manipulator OBJ name
                    string tName = argv[++i];
                    mObj_fName = tName;
                    optList["-m"] = 1;
                } else {
                    cerr << "** ERROR! No filename provided for the -m switch! **\n" << endl;
                    print_usage();
                    return 1;
                }
                break;
            case 'c':
                if (i + 1 < argc) { // look for the cockpit OBJ name
                    string tName = argv[++i];
                    cObj_fName = tName;
                    optList["-c"] = 1;
                } else {
                    cerr << "** ERROR! No filename provided for the -c switch! **\n" << endl;
                    print_usage();
                    return 1;
                }
                break;
            default:
                // invalid switch is sent!
                cerr << "** ERROR! Unrecognized switch: " << arg << " **\n" << endl;
                print_usage();
                return 1;
        }
    }

    int missing_options = 0;  // we'll assume the user provided all the required options (but we'll verify just in case)
    for (pair<string, int> optCheck:optList){
        if (optCheck.second == 0) { // oops! missing a required option!
            cerr << "** ERROR! Missing " << optCheck.first << " option!" << endl;
            missing_options = 1;
        } 
    }
    if (missing_options == 1) { // looks like they forgot something so we'll show them the usage.
        cerr << endl;
        print_usage();
        return 1;
    }

    return 0;
}

int main(int argc, char* argv[]) {

    string kb_title = "KITBASH 2.0 ver " + VERSION; // title string for output
    if (kb_debug) {
        kb_title += " ***DEBUG MODE***";
    }

    string acf_fName = "";          // full path and name of ACF file
    string pObj_name = "";          // name of positioned OBJ to find in the acf file
    string mObj_fName = "";          // full path and name of related manipulator obj file that controls the positioned OBJ
    string cObj_fName = "";          // full path and name of the cockpit obj file to modify
    cout << "\n" << kb_title << "\n" << endl;

if (arg_handler(argc, argv, acf_fName, pObj_name, mObj_fName, cObj_fName) == 1) {return 1;};
 
    xp_acf_file acf_file;
    if (!(acf_file.set_acf_fName(acf_fName)==1)) {
        cerr    << "** ERROR! Unable to find and/or open ACF File: " << acf_fName << endl;
        return 1;
    }

    xp_manip_file manip_file;
    if (!(manip_file.set_manip_fName(mObj_fName)==1)) {
        cerr    << "** ERROR! Unable to find and/or open manipulator OBJ File: " << mObj_fName << endl;
        return 1;
    }

    xp_cockpit_file cockpit_file;
    if (!(cockpit_file.set_cockpit_fName(cObj_fName)==1)) {
        cerr << "**ERROR! Unable to find and/or open cockpit OBJ file: " << cObj_fName << endl;
        return 1;
    }

    cout    << "ACF File:\t\t" << acf_fName << "\n"
            << "Positioned OBJ:\t\t" << pObj_name << "\n" 
            << "Manipulator OBJ:\t" << mObj_fName << "\n"
            << "Cockpit OBJ:\t\t" << cObj_fName << endl;
    if (!ow_switch) {
        cout << "\nVerify file names and locations and type [Y]es to proceed with kitbashing!: ";
        string input_string;
        getline (cin, input_string);
        char *input_chars = &input_string[0];
        char input_char = tolower(input_chars[0]);
        if (input_char != 'y') {
            cerr << "\nProcess stopped by user.  Enjoy the rest of your day!" << endl;
            return 1;
        }
    }
    auto start_time = Clock::now();
    cout    << "\nKitbashing commences!  Please stand by...\n" << endl;
    if (acf_file.set_pObj_fName (pObj_name) == 0) {
        cerr    << "\nPositioned OBJ file [" << pObj_name << "] not found in " << acf_fName << "\n"
                << "Kitbashing aborted.  Please verify the file has been positioned and try again.\n" << endl;
        return 1;
    }
    cout    << pObj_name << " found in " << acf_fName << "\n"
            << "Psi (yaw) rotation:\t" << fixed << setprecision (6) << acf_file.pObj_rotation_psi << "\n"
            << "Theta (pitch) rotation:\t" << acf_file.pObj_rotation_theta << "\n"
            << "Phi (roll) rotation:\t" << acf_file.pObj_rotation_phi << "\n"
            << "X axis offset:\t\t" << fixed << setprecision (8) << acf_file.pObj_offset_x << "\n"
            << "Y axis offset:\t\t" << acf_file.pObj_offset_y << "\n"
            << "Z axis offset:\t\t" << acf_file.pObj_offset_z << "\n"
            << endl;

    // read the manipulator.obj file and rotationally and axially transform the VTs from rotational and offset
    // data gleaned from the acf file.
    manip_file.transform_vts (&acf_file);

    // read the cockpit file.
    int err = 0;
    do {
        err = cockpit_file.read_xp_cockpit_file(pObj_name, &manip_file, ow_switch);
        switch (err) {
            case 0:
                cout    << cObj_fName << " summary\n"
                        << "Orig VTs:\t\t" << cockpit_file.get_vt_count(0) << "\n"
                        << "Added VTs:\t\t" << cockpit_file.get_vt_count(1) << "\n"
                        << "Total VTs:\t\t" << cockpit_file.get_vt_count(2) << "\n"
                        << "Orig TRIS:\t\t" << cockpit_file.get_idx_count() << "\n"
                        << "Added TRIS:\t\t" << cockpit_file.get_idx_count(1) << "\n"
                        << "Total TRIS:\t\t" << cockpit_file.get_idx_count(2) << "\n"
                        << "----------------------------------\n"
                        << "Last VT line:\t\t" << cockpit_file.get_orig_vt_end_index()+1 << "\n"
                        << "Last IDX/IDX10 line:\t" << cockpit_file.get_orig_idx_end_index()+1 << "\n"
                        << endl;
                break;
            case 1:
                cerr << "** ERROR! Unable to open cockpit OBJ file. Process stopped." << endl;
                return 1;
                break;
            case 2:
                cerr << "** ERROR! Unable to rename cockpit OBJ file to .SAVED.  Process stopped." << endl;
                return 1;
                break;
            case 4:
                cerr << "** ERROR! Unable to analyze cockpit OBJ file. Process stopped." << endl;
                return 1;
                break;
            case 3:
                cout << pObj_name << " already appended to the cockpit object specified.  Overwrite? (y/N): ";
                string input_string;
                getline (cin, input_string);
                char *input_chars = &input_string[0];
                char input_char = tolower(input_chars[0]);
                if (input_char != 'y') {
                    cerr << "\nProcess stopped by user.  Enjoy the rest of your day!" << endl;
                    return 1;
                }
                ow_switch = true;
        }
    } while (err > 2); // if we want to overwrite we need to loop it again.
            
    auto end_time = Clock::now();
    auto elapsed_time = chrono::duration_cast<float_seconds> (end_time - start_time);
    cout    << "Completed in: " << fixed << setprecision(4) << elapsed_time.count() << " seconds." << endl;
    cout    << "And Milli's your aunt." << endl;

}

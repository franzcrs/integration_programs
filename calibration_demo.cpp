#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <variant>

using namespace std;

/* Adding functionality */
template <typename S>
ostream& operator<<(ostream& os,
                    const vector<S>& vector)
{
    // Printing all the elements
    // using <<
    for (auto element : vector) {
        os << element << " ";
    }
    return os;
}

/* Dictionary of Joint Motions */

// Using library //
#include "json.hpp"
using json = nlohmann::json;
// std::ifstream jsonStream("joint_motions.json"); // Load JSON file data
// json jointMotions = json::parse(jsonStream); // Parse data into json object

// Using Hash Map //
typedef map <string, map<string, variant< double, vector<string>, vector<double> >>> Dict;
Dict jointMotions = {
    {"elbow_flexion", {
        {"f_ratio", 1.1616},
        {"muscles", vector<string> {"BICLong", "BICShort", "BRA", "BRD"}},
        {"proportion", vector<double> {1, 0.603, 2.24, 0.525}}
        }
    },
    {"elbow_extension", {
        {"f_ratio", 1.0},
        {"muscles", vector<string> {"TRILong", "TRILat", "TRIMed"}},
        {"proportion", vector<double> {1, 0.929, 0.929}}
        }
    },
    {"hand_grip", {
        {"f_ratio", 1.1227},
        {"muscles", vector<string> {"CC", "DD"}},
        {"proportion", vector<double> {1, 0.7}}
        }
    }
};

/* Functions */
double maxJointTorque(string motionName, double torqueAtMet , double areaCurveTorqueUntilMet){
    // Parameters for Newton's method
    double initValue = 0.1;
    double delta = 0.000000001;
    int maxIts = 100;
    int i;

    // Variables for formula
    double λF = 0;
    try {
        // Get fatigue ratio from the dicitonary of joint motions
        λF = std::get<double>(jointMotions[motionName]["f_ratio"]);
    }
    catch (const std::out_of_range& e) {
        // Explain error and return -1
        std::cerr << "＞ The provided motion name: \"" << motionName << "\" does not exist in the dictionary of motions" << std::endl;
        return -1; // Return -1 to indicate an error
    }
    double maxTorqPrev = 0;
    double maxTorq = initValue;

    // Newton Method for finding roots
    for (i = 0; i < maxIts; i++) {
        maxTorqPrev = maxTorq;
        // X update: x = x - (f(x)/f'(x))
        maxTorq = maxTorq - (log(torqueAtMet / maxTorq) + (λF * areaCurveTorqueUntilMet / maxTorq)) / ((-1) * (1/maxTorq + λF*areaCurveTorqueUntilMet/(maxTorq * maxTorq)));

        // Convergence method: |x - xPrev|/|xPrev| < delta
        if (fabs(maxTorq - maxTorqPrev) / fabs(maxTorqPrev) < delta) {
            // std::cout << "＞ Root value converged!" << std::endl;
            // std::cout << "＞ τ_max = " << std::round(maxTorq * 100) / 100.0 << std::endl << std::endl;
            return maxTorq;
        }
    }
    if (i == maxIts) {
        std::cerr << "＞ Root value could not converge in the defined number of iterations" << std::endl << std::endl;
        return -1; // Return -1 to indicate an error
    }

    return -1; // Return -1 to indicate an error
}

vector<double> musclesMvc(string motionName, double maxJointTorque, map <string, double> momentArms){
    // Variables for method
    vector<string> muscles;
    vector<double> proportions;
    try {
        // Get list of participants muscles
        muscles = std::get<vector<string>>(jointMotions[motionName]["muscles"]);
        // Get list of maximum force proportion
        proportions = std::get<vector<double>>(jointMotions[motionName]["proportion"]);
    }
    catch (const std::out_of_range& e) {
        // Explain error and return empty
        std::cerr << "＞ The provided motion name: \"" << motionName << "\" does not exist in the dictionary of motions" << std::endl;
        return {}; // Return empty to indicate an error
    }
    
    // Solution
    vector<double> mvcPerMuscle; // output
    double sumPropXArms = 0; // sum of proportions x arms
    int i;
    try {
        for (i =0; i<int(muscles.size()); i++){
            sumPropXArms = sumPropXArms + proportions[static_cast<unsigned long>(i)]*momentArms[muscles[static_cast<unsigned long>(i)]];
        }
    }
    catch (const std::out_of_range& e){
        // Explain error and return empty
        std::cerr << "＞ The provided muscle name: \"" << muscles[static_cast<unsigned long>(i)] << "\" does not exist in the dictionary of moment arms" << std::endl;
        return {}; // Return empty to indicate an error
    }
    // Reference muscle's MVC (geometric prop: 1, and first in proportions list in joints motion)
    double refMuscleMvc = maxJointTorque/sumPropXArms;
    // Rest of muscles' MVC
    for (double proportion: proportions) {
        mvcPerMuscle.push_back(refMuscleMvc*proportion);
    }
    return mvcPerMuscle;
}

vector<double> musclesFatigueRatio(vector<double> musclesMvcList, vector<double> forcesAtMetFiltered, vector<double> areaCurveForceUntilMetFiltered){
    vector<double> musclesFR;
    for (int i=0; i<int(musclesMvcList.size()); i++){
        double forceMuscle = forcesAtMetFiltered[static_cast<unsigned long>(i)];
        double mvcMuscle = musclesMvcList[static_cast<unsigned long>(i)];
        double areaMuscle = areaCurveForceUntilMetFiltered[static_cast<unsigned long>(i)];
        if (forceMuscle >= mvcMuscle) {
            std::cerr << "The provided force(met): " << forceMuscle << "and MVC: " << mvcMuscle << ", for the muscle of index ";
            std::cerr << static_cast<double>(i) << " does not comply with the condition: force(met)<MVC" << std::endl << std::endl;
            return {};
        }
        musclesFR.push_back((-1)*log(forceMuscle/mvcMuscle)*mvcMuscle/areaMuscle);
    }
    return musclesFR;
}

/* Main */
int main(){
    /* Demo variables */

    /* ***************** Calibration data of isometric exercises ***************** */
    map <string, map < string, variant< double, map <string, double> > >> calibrationData = {
        // Elbow flexion calibration data
        {"elbow_flexion", {
            {"torque_at_met", 20.}, // Joint torque at Maximum Endurance Time(MET)
            {"area_curve_torque_until_met", 700.}, // Area under the curve of the joint torque from t=0 until MET
            {"all_forces_at_met", map <string, double>{ // List of all muscles forces at MET obtained from model simulations
                {"BICLong", 200},
                {"BICShort", 100},
                {"BRA", 400},
                {"BRD", 100},
                {"TRILong", 0},
                {"TRILat", 0},
                {"TRIMed", 0}
                }
            },
            {"all_area_curve_force_until_met", map <string, double>{ // List of all muscles forces area under the curve from t=0 until MET
                {"BICLong", 1800},
                {"BICShort", 1200},
                {"BRA", 3600},
                {"BRD", 1200},
                {"TRILong", 0},
                {"TRILat", 0},
                {"TRIMed", 0}
                }
            },
            {"moment_arms", map <string, double>{ // ist of all muscles moment arms during the isometric exercise
                {"BICLong", 0.05,},
                {"BICShort", 0.05},
                {"BRA", 0.02},
                {"BRD", 0.08}
                }
            }
            }
        },
        // Add here other joint motion calibration data
    };
    
    /* ***************** Access demo for jointMotions dictionary values ***************** */
    // {
    // printf("Access demo for jointMotions dictionary values \n");
    // // In case of using json library
    // // std::cout << "jointMotions[motionName][\"muscles\"][0]" << endl;
    // // cout << jointMotions[motionName]["muscles"][0] << endl;
    // // In case of using hash map
    // std::cout << "std::get<vector<string>>(jointMotions[motionName][\"muscles\"])[0]" << endl;
    // std::cout << std::get<vector<string>>(jointMotions[motionName]["muscles"])[0] << endl << endl;
    // };

    
    /* Execution */

    /* ***************** Dictionary of Computed muscles parameters ***************** */
    map<string, vector<double>> musclesParameters;
    
    /* ***************** Iteration among isometric exercises calibration data ***************** */
    for (const auto& [motionName, data] : calibrationData) {
        std::cout << "Computation results :" << endl;
        // 1. Calculate the Maximum joint torque for the calibrated joints
        double maxJointTorq = maxJointTorque(motionName, std::get<double>(calibrationData[motionName]["torque_at_met"]), std::get<double>(calibrationData[motionName]["area_curve_torque_until_met"]));
        std::cout << "Maximum joint torque for: " << motionName << ", is : "  << maxJointTorq << endl;
        // 2. Distribute the MVC among the participants muscles
        vector<string> muscles = std::get<vector<string>>(jointMotions[motionName]["muscles"]);// Get list of participants muscles
        std::cout << "Participants muscles : { " << muscles << "}" << endl;
        vector<double> musclesMvcList = musclesMvc(motionName, maxJointTorq, std::get<map <string, double>>(calibrationData[motionName]["moment_arms"]));
        std::cout << "Muscles' MVC: { " << musclesMvcList << "}" << endl;
        // 3. Calculate the fatigue ratio per participant muscle
        string temp;
        vector<double> forcesAtMetFiltered;
        vector<double> areaCurveForceUntilMetFiltered;
        try {
            for (string muscle: std::get<vector<string>>(jointMotions[motionName]["muscles"])){
                // std::cout << "Test " << std::get<map<string, double>>(calibrationData[motionName]["all_forces_at_met"])[muscle] << endl;
                forcesAtMetFiltered.push_back(std::get<map<string, double>>(calibrationData[motionName]["all_forces_at_met"])[muscle]);
                areaCurveForceUntilMetFiltered.push_back(std::get<map <string, double>>(calibrationData[motionName]["all_area_curve_force_until_met"])[muscle]);
                temp = muscle;
            }
        }
        catch (const std::out_of_range& e){
            // Explain error and return empty
            std::cerr << "＞ The provided muscle name: \"" << temp << "\" does not exist in the dictionaries of forces or areas" << std::endl;
            return 0; // Return 0 to indicate an error
        }
        vector<double> musclesFatigueRatioList = musclesFatigueRatio(musclesMvcList, forcesAtMetFiltered, areaCurveForceUntilMetFiltered);
        std::cout << "Muscles' λF: { " << musclesFatigueRatioList << "}" << endl << endl;
        // 4. Store the values in the musclesParameter dictionary (Keeping the highest MVC if muscle repeats)
        for (unsigned long i=0; i<muscles.size(); i++) {
            // Check if muscle's parameters already exists in the musclesParameters dictionary
            if (musclesParameters.count(muscles[i]) > 0) {
                double prevMvc = musclesParameters[muscles[i]][0];
                // double prevλF = musclesParameters[muscles[i]][1];
                if (prevMvc > musclesMvcList[i]){
                    continue;
                }
            }
            musclesParameters.insert({muscles[i], vector<double>{musclesMvcList[i], musclesFatigueRatioList[i]}});
        }
    }

    /* ***************** Resulting dictionary of muscle's parameter ***************** */
    std::cout << "Muscles Parameters : " << endl;
    int lines = 0;
    for (const auto& [key, value] : musclesParameters) {
        lines++;
        std::cout << "Key: " << key << ", Value: {" << value << "}" << endl;
    }
    std::cout << endl;
    
    /* ***************** Exporting dictionary to JSON file ***************** */
    {
    // Convert map into json object
    json jsonData;
    for (const auto& pair : musclesParameters) {
        jsonData[pair.first] = pair.second;
    }
    // Write the JSON data to a file
    std::ofstream outputFile("muscles_parameters.json");
    outputFile << jsonData.dump(lines); // The argument indents the output for readability
    };

    std::cout << endl;
    return 0;
}
// TODO: update the json files and dicitonary declare inside program

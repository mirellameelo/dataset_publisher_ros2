#include "seeds_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <string>
#include <iostream>

seeds_sequence::seeds_sequence(const std::string& seq_dir_path) {

    // load timestamps
    const std::string timestamp_file_path_left = seq_dir_path + "/left.txt";
    const std::string timestamp_file_path_right = seq_dir_path + "/right.txt";
    std::ifstream ifs_timestamp_left, ifs_timestamp_right;
    ifs_timestamp_left.open(timestamp_file_path_left.c_str());
    ifs_timestamp_right.open(timestamp_file_path_right.c_str());
    if (!ifs_timestamp_left || !ifs_timestamp_right) {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path_left);
    }

    timestamps_left.clear();
    timestamps_right.clear();
    while (!ifs_timestamp_left.eof()) {
        std::string s;
        getline(ifs_timestamp_left, s);
        if (!s.empty()) {
            timestamps_left.push_back(s);
        }
    }
    while (!ifs_timestamp_right.eof()) {
        std::string s;
        getline(ifs_timestamp_right, s);
        if (!s.empty()) {
            timestamps_right.push_back(s);
        }
    }

    ifs_timestamp_left.close();
    ifs_timestamp_right.close();

    // load image file paths
    const std::string left_img_dir_path = seq_dir_path + "/cam0/data/";
    const std::string right_img_dir_path = seq_dir_path + "/cam1/data/";

    left_img_file_paths_.clear();
    right_img_file_paths_.clear();

    for (unsigned int i = 0; i < timestamps_left.size(); ++i) {
        std::stringstream ss;
            ss << "SVS_L_" << timestamps_left[i];
            left_img_file_paths_.push_back(left_img_dir_path + ss.str() + ".png");
    }

    for (unsigned int i = 0; i < timestamps_right.size(); ++i) {
        std::stringstream ss;
        ss << "SVS_R_" << timestamps_right[i];
        right_img_file_paths_.push_back(right_img_dir_path + ss.str() + ".png");
    }
}

std::vector<seeds_sequence::frame> seeds_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < left_img_file_paths_.size(); ++i) {
        frames.emplace_back(  frame{  left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_left.at(i), timestamps_right.at(i) });
    }
    return frames;
}

#ifndef EXAMPLE_UTIL_KITTI_UTIL_H
#define EXAMPLE_UTIL_KITTI_UTIL_H

#include <string>
#include <vector>

class seeds_sequence {
public:
    struct frame {
        frame(const std::string& left_img_path, const std::string& right_img_path, 
                const std::string timestamp_left, const std::string timestamp_right)
            : left_img_path_(left_img_path), right_img_path_(right_img_path), timestamp_left_(timestamp_left), timestamp_right_(timestamp_right){};

        const std::string left_img_path_;
        const std::string right_img_path_;
        const std::string timestamp_left_;
        const std::string timestamp_right_;
    };

    explicit seeds_sequence(const std::string& seq_dir_path);

    virtual ~seeds_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<std::string> timestamps_left;
    std::vector<std::string> timestamps_right;
    std::vector<std::string> left_img_file_paths_;
    std::vector<std::string> right_img_file_paths_;
};

#endif // EXAMPLE_UTIL_KITTI_UTIL_H

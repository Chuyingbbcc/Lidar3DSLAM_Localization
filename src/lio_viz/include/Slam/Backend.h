//
// Created by chuchu on 6/1/26.
//
#include <string>
#include <vector>
#include <memory>

class KeyFrame;
class Backend {
public:
    Backend(const std::string& init_path);
    ~Backend() = default;
    void addKeyFrame(std::shared_ptr<KeyFrame> kf);

private:
    std::vector<std::shared_ptr<KeyFrame>> key_frames_;
    void processNewKeyFrame(std::shared_ptr<KeyFrame> kf);

    void runLevel1Optimization();
    void runLevel2Optimization();
};


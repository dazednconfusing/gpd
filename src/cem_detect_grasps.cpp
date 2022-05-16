#include <string>

#include <gpd/sequential_importance_sampling.h>

namespace gpd {
namespace apps {
namespace detect_grasps {

bool checkFileExists(const std::string &file_name) {
  std::ifstream file;
  file.open(file_name.c_str());
  if (!file) {
    std::cout << "File " + file_name + " could not be found!\n";
    return false;
  }
  file.close();
  return true;
}
namespace
{
void quaternionFromRPY(const double r, const double p, const double y, Eigen::Quaterniond* const q)
{
  Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

  // *q = rollAngle * pitchAngle * yawAngle;
  *q = yawAngle * pitchAngle * rollAngle;
};

void quaternionFromRPY(const std::vector<double>& rpy, Eigen::Quaterniond* const q)
{
  quaternionFromRPY(rpy[0], rpy[1], rpy[2], q);
}

Eigen::Isometry3d OptToBaseFrame(const Eigen::Vector3d& base_cam_trans_,
  const Eigen::VectorXd& base_cam_rot_,
  const Eigen::Vector3d& cam_opt_trans_,
  const Eigen::VectorXd& cam_opt_rot_)
{
  Eigen::Quaterniond cam_opt_q(cam_opt_rot_[0], cam_opt_rot_[1], cam_opt_rot_[2], cam_opt_rot_[3]);
  Eigen::Quaterniond base_cam_q(base_cam_rot_[0], base_cam_rot_[1], base_cam_rot_[2], base_cam_rot_[3]);
  return (Eigen::Translation3d(base_cam_trans_) * base_cam_q) * (Eigen::Translation3d(cam_opt_trans_) * cam_opt_q);
}
} // namespace

int DoMain(int argc, char *argv[]) {
  // Read arguments from command line.
  if (argc < 3) {
    std::cout << "Error: Not enough input arguments!\n\n";
    std::cout
        << "Usage: cem_detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
    std::cout << "Detect grasp poses for a point cloud, PCD_FILE (*.pcd), "
                 "using parameters from CONFIG_FILE (*.cfg).\n\n";
    std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each "
                 "point in the cloud (*.csv).\n";
    return (-1);
  }

  std::string config_filename = argv[1];
  std::string pcd_filename = argv[2];
  if (!checkFileExists(config_filename)) {
    printf("Error: config file not found!\n");
    return (-1);
  }
  if (!checkFileExists(pcd_filename)) {
    printf("Error: PCD file not found!\n");
    return (-1);
  }

  // View point from which the camera sees the point cloud.
  Eigen::Matrix3Xd view_points(3, 1);
  view_points.setZero();

  // Load point cloud from file
  util::Cloud cloud(pcd_filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  // Load surface normals from file.
  if (argc > 3) {
    std::string normals_filename = argv[3];
    cloud.setNormalsFromFile(normals_filename);
    std::cout << "Loaded surface normals from file: " << normals_filename
              << "\n";
  }

  // Read parameters from configuration file.
  const double VOXEL_SIZE = 0.003;
  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();
  std::vector<double> workspace =
      config_file.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");
  int num_threads = config_file.getValueOfKey<int>("num_threads", 1);
  bool sample_above_plane =
      config_file.getValueOfKey<int>("sample_above_plane", false);
  double normals_radius =
      config_file.getValueOfKey<double>("normals_radius", 0.03);
  printf("num_threads: %d\n", num_threads);
  printf("sample_above_plane: %d\n", sample_above_plane);
  printf("normals_radius: %.3f\n", normals_radius);
  // Camera transforms
  std::vector<double> tf_base_cam =
    config_file.getValueOfKeyAsStdVectorDouble("tf_base_cam", "0, 0, 0.57, 0, 0.707, 0");
  std::vector<double> tf_cam_opt =
    config_file.getValueOfKeyAsStdVectorDouble("tf_cam_opt", "0, 0, 0, -1.57, 0, -1.57");
  Eigen::Vector3d base_cam_trans_;
  Eigen::VectorXd base_cam_rot_;
  Eigen::Vector3d cam_opt_trans_;
  Eigen::VectorXd cam_opt_rot_;

  base_cam_trans_ << tf_base_cam[0], tf_base_cam[1], tf_base_cam[2];
  Eigen::Quaterniond base_cam_q;
  quaternionFromRPY(tf_base_cam[3], tf_base_cam[4], tf_base_cam[5], &base_cam_q);
  base_cam_rot_.resize(4);
  base_cam_rot_ << base_cam_q.w(), base_cam_q.x(), base_cam_q.y(), base_cam_q.z();

  cam_opt_trans_ << tf_cam_opt[0], tf_cam_opt[1], tf_cam_opt[2];
  Eigen::Quaterniond cam_opt_q;
  quaternionFromRPY(tf_cam_opt[3], tf_cam_opt[4], tf_cam_opt[5], &cam_opt_q);
  cam_opt_rot_.resize(4);
  cam_opt_rot_ << cam_opt_q.w(), cam_opt_q.x(), cam_opt_q.y(), cam_opt_q.z();

  // Preprocess the point cloud.
  Eigen::Isometry3d base_opt_trans = OptToBaseFrame(base_cam_trans_,
    base_cam_rot_,
    cam_opt_trans_,
    cam_opt_rot_);
  // Preprocess the point cloud.
  cloud.filterWorkspace(workspace, base_opt_trans);
  cloud.voxelizeCloud(VOXEL_SIZE);
  cloud.calculateNormals(num_threads, normals_radius);
  if (sample_above_plane) {
    cloud.sampleAbovePlane();
  }

  // Detect grasp affordances.
  SequentialImportanceSampling detector(config_filename);
  detector.detectGrasps(cloud);

  return 0;
}

}  // namespace detect_grasps
}  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  return gpd::apps::detect_grasps::DoMain(argc, argv);
}

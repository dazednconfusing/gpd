#include <gpd/grasp_detector.h>
namespace gpd
{
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
} // namespace

GraspDetector::GraspDetector(const std::string& config_filename)
{
  Eigen::initParallel();

  // Read parameters from configuration file.
  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();

  // Read hand geometry parameters.
  std::string hand_geometry_filename =
    config_file.getValueOfKeyAsString("hand_geometry_filename", "");
  if (hand_geometry_filename == "0")
  {
    hand_geometry_filename = config_filename;
  }
  candidate::HandGeometry hand_geom(hand_geometry_filename);
  std::cout << hand_geom;

  // Read plotting parameters.
  plot_normals_ = config_file.getValueOfKey<bool>("plot_normals", false);
  plot_samples_ = config_file.getValueOfKey<bool>("plot_samples", true);
  plot_candidates_ = config_file.getValueOfKey<bool>("plot_candidates", false);
  plot_filtered_candidates_ =
    config_file.getValueOfKey<bool>("plot_filtered_candidates", false);
  plot_valid_grasps_ =
    config_file.getValueOfKey<bool>("plot_valid_grasps", false);
  plot_clustered_grasps_ =
    config_file.getValueOfKey<bool>("plot_clustered_grasps", false);
  plot_selected_grasps_ =
    config_file.getValueOfKey<bool>("plot_selected_grasps", false);
  printf("============ PLOTTING ========================\n");
  printf("plot_normals: %s\n", plot_normals_ ? "true" : "false");
  printf("plot_samples %s\n", plot_samples_ ? "true" : "false");
  printf("plot_candidates: %s\n", plot_candidates_ ? "true" : "false");
  printf("plot_filtered_candidates: %s\n",
    plot_filtered_candidates_ ? "true" : "false");
  printf("plot_valid_grasps: %s\n", plot_valid_grasps_ ? "true" : "false");
  printf("plot_clustered_grasps: %s\n",
    plot_clustered_grasps_ ? "true" : "false");
  printf("plot_selected_grasps: %s\n",
    plot_selected_grasps_ ? "true" : "false");
  printf("==============================================\n");

  // Create object to generate grasp candidates.
  candidate::CandidatesGenerator::Parameters generator_params;
  generator_params.num_samples_ =
    config_file.getValueOfKey<int>("num_samples", 1000);
  generator_params.num_threads_ =
    config_file.getValueOfKey<int>("num_threads", 1);
  generator_params.remove_statistical_outliers_ =
    config_file.getValueOfKey<bool>("remove_outliers", false);
  generator_params.sample_above_plane_ =
    config_file.getValueOfKey<bool>("sample_above_plane", false);
  generator_params.voxelize_ =
    config_file.getValueOfKey<bool>("voxelize", true);
  generator_params.voxel_size_ =
    config_file.getValueOfKey<double>("voxel_size", 0.003);
  generator_params.normals_radius_ =
    config_file.getValueOfKey<double>("normals_radius", 0.03);
  generator_params.refine_normals_k_ =
    config_file.getValueOfKey<int>("refine_normals_k", 0);
  generator_params.workspace_ =
    config_file.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");

  candidate::HandSearch::Parameters hand_search_params;
  hand_search_params.hand_geometry_ = hand_geom;
  hand_search_params.nn_radius_frames_ =
    config_file.getValueOfKey<double>("nn_radius", 0.01);
  hand_search_params.num_samples_ =
    config_file.getValueOfKey<int>("num_samples", 1000);
  hand_search_params.num_threads_ =
    config_file.getValueOfKey<int>("num_threads", 1);
  hand_search_params.num_orientations_ =
    config_file.getValueOfKey<int>("num_orientations", 8);
  hand_search_params.num_finger_placements_ =
    config_file.getValueOfKey<int>("num_finger_placements", 10);
  hand_search_params.deepen_hand_ =
    config_file.getValueOfKey<bool>("deepen_hand", true);
  hand_search_params.hand_axes_ =
    config_file.getValueOfKeyAsStdVectorInt("hand_axes", "2");
  hand_search_params.friction_coeff_ =
    config_file.getValueOfKey<double>("friction_coeff", 20.0);
  hand_search_params.min_viable_ =
    config_file.getValueOfKey<int>("min_viable", 6);
  candidates_generator_ = std::make_unique<candidate::CandidatesGenerator>(
    generator_params, hand_search_params);

  // Camera transforms
  std::vector<double> tf_base_cam =
    config_file.getValueOfKeyAsStdVectorDouble("tf_base_cam", "0, 0, 0.57, 0, 0.707, 0");
  std::vector<double> tf_cam_opt =
    config_file.getValueOfKeyAsStdVectorDouble("tf_cam_opt", "0, 0, 0, -1.57, 0, -1.57");

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
  printf("============ TRANSFORMS =============\n");
  printf("Transform cam to opt q: w: %.2f x: %.2f y: %.2f z: %.2f\n",
    cam_opt_q.w(), cam_opt_q.x(), cam_opt_q.y(), cam_opt_q.z());
  printf("Transform base to cam q: w: %.2f x: %.2f y: %.2f z: %.2f\n",
    base_cam_q.w(), base_cam_q.x(), base_cam_q.y(), base_cam_q.z());
  printf("Transform cam to opt quaternion: w: %.2f x: %.2f y: %.2f z: %f\n",
    cam_opt_rot_[0], cam_opt_rot_[1], cam_opt_rot_[2], cam_opt_rot_[3]);
  printf("Transform base to cam quaternion: w: %.2f x: %.2f y: %.2f z: %f\n",
    base_cam_rot_[0], base_cam_rot_[1], base_cam_rot_[2], base_cam_rot_[3]);
  printf("============ CLOUD PREPROCESSING =============\n");
  printf("voxelize: %s\n", generator_params.voxelize_ ? "true" : "false");
  printf("voxel_size: %.3f\n", generator_params.voxel_size_);
  printf("remove_outliers: %s\n",
    generator_params.remove_statistical_outliers_ ? "true" : "false");
  printStdVector(generator_params.workspace_, "workspace");
  printf("sample_above_plane: %s\n",
    generator_params.sample_above_plane_ ? "true" : "false");
  printf("normals_radius: %.3f\n", generator_params.normals_radius_);
  printf("refine_normals_k: %d\n", generator_params.refine_normals_k_);
  printf("==============================================\n");

  printf("============ CANDIDATE GENERATION ============\n");
  printf("num_samples: %d\n", hand_search_params.num_samples_);
  printf("num_threads: %d\n", hand_search_params.num_threads_);
  printf("nn_radius: %3.2f\n", hand_search_params.nn_radius_frames_);
  printStdVector(hand_search_params.hand_axes_, "hand axes");
  printf("num_orientations: %d\n", hand_search_params.num_orientations_);
  printf("num_finger_placements: %d\n",
    hand_search_params.num_finger_placements_);
  printf("deepen_hand: %s\n",
    hand_search_params.deepen_hand_ ? "true" : "false");
  printf("friction_coeff: %3.2f\n", hand_search_params.friction_coeff_);
  printf("min_viable: %d\n", hand_search_params.min_viable_);
  printf("==============================================\n");

  // TODO: Set the camera position.
  //  Eigen::Matrix3Xd view_points(3,1);
  //  view_points << camera_position[0], camera_position[1], camera_position[2];

  // Read grasp image parameters.
  std::string image_geometry_filename =
    config_file.getValueOfKeyAsString("image_geometry_filename", "");
  if (image_geometry_filename == "0")
  {
    image_geometry_filename = config_filename;
  }
  descriptor::ImageGeometry image_geom(image_geometry_filename);
  std::cout << image_geom;

  // Read classification parameters and create classifier.
  std::string model_file = config_file.getValueOfKeyAsString("model_file", "");
  std::string weights_file =
    config_file.getValueOfKeyAsString("weights_file", "");
  if (!model_file.empty() || !weights_file.empty())
  {
    int device = config_file.getValueOfKey<int>("device", 0);
    int batch_size = config_file.getValueOfKey<int>("batch_size", 1);
    classifier_ = net::Classifier::create(
      model_file, weights_file, static_cast<net::Classifier::Device>(device),
      batch_size);
    min_score_ = config_file.getValueOfKey<int>("min_score", 0);
    printf("============ CLASSIFIER ======================\n");
    printf("model_file: %s\n", model_file.c_str());
    printf("weights_file: %s\n", weights_file.c_str());
    printf("batch_size: %d\n", batch_size);
    printf("==============================================\n");
  }

  // Read additional grasp image creation parameters.
  bool remove_plane = config_file.getValueOfKey<bool>(
    "remove_plane_before_image_calculation", false);

  // Create object to create grasp images from grasp candidates (used for
  // classification).
  image_generator_ = std::make_unique<descriptor::ImageGenerator>(
    image_geom, hand_search_params.num_threads_,
    hand_search_params.num_orientations_, false, remove_plane);

  // Read grasp filtering parameters based on robot workspace and gripper width.
  workspace_grasps_ = config_file.getValueOfKeyAsStdVectorDouble(
    "workspace_grasps", "-1 1 -1 1 -1 1");
  min_aperture_ = config_file.getValueOfKey<double>("min_aperture", 0.0);
  max_aperture_ = config_file.getValueOfKey<double>("max_aperture", 0.045);
  printf("============ CANDIDATE FILTERING =============\n");
  printStdVector(workspace_grasps_, "candidate_workspace");
  printf("min_aperture: %3.4f\n", min_aperture_);
  printf("max_aperture: %3.4f\n", max_aperture_);
  printf("==============================================\n");

  // Read grasp filtering parameters based on approach direction.
  filter_approach_direction_ =
    config_file.getValueOfKey<bool>("filter_approach_direction", false);
  std::vector<double> approach =
    config_file.getValueOfKeyAsStdVectorDouble("direction", "1 0 0");
  thresh_rad_ = config_file.getValueOfKey<double>("thresh_rad", 2.3);

  // Actual new change after fork start
  // Perform transform to optical frame
  Eigen::Vector3d direction = BaseToOptFrame(approach);
  direction_ << direction[0], direction[1], direction[2];
  Eigen::Vector3d directionToBase = OptToBaseFrame(direction);
  printf("============ APPROACH ======================\n");
  printf("filter approach: %s\n", filter_approach_direction_ ? "true" : "false");
  printf("approach: base frame= %.2f %.2f %.2f, opt frame= %.2f %.2f %.2f\n ",
    approach[0], approach[1], approach[2],
    direction_[0], direction_[1], direction_[2]);
  printf("back to base= %.2f %.2f %.2f\n ", directionToBase[0], directionToBase[1], directionToBase[2]);
  printf("==============================================\n\n");

  // Actual mew change after fork end

  // Read clustering parameters.
  int min_inliers = config_file.getValueOfKey<int>("min_inliers", 1);
  clustering_ = std::make_unique<Clustering>(min_inliers);
  cluster_grasps_ = min_inliers > 0 ? true : false;
  printf("============ CLUSTERING ======================\n");
  printf("min_inliers: %d\n", min_inliers);
  printf("==============================================\n\n");

  // Read grasp selection parameters.
  num_selected_ = config_file.getValueOfKey<int>("num_selected", 100);

  // Create plotter.
  plotter_ = std::make_unique<util::Plot>(hand_search_params.hand_axes_.size(),
    hand_search_params.num_orientations_);
}

std::vector<std::unique_ptr<candidate::Hand>> GraspDetector::detectGrasps(
  const util::Cloud& cloud)
{
  double t0_total = omp_get_wtime();
  std::vector<std::unique_ptr<candidate::Hand>> hands_out;

  const candidate::HandGeometry& hand_geom =
    candidates_generator_->getHandSearchParams().hand_geometry_;

  // Check if the point cloud is empty.
  if (cloud.getCloudOriginal()->size() == 0)
  {
    printf("ERROR: Point cloud is empty!");
    hands_out.resize(0);
    return hands_out;
  }

  // Plot samples/indices.
  if (plot_samples_)
  {
    if (cloud.getSamples().cols() > 0)
    {
      plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
    }
    else if (cloud.getSampleIndices().size() > 0)
    {
      plotter_->plotSamples(cloud.getSampleIndices(),
        cloud.getCloudProcessed());
    }
  }

  if (plot_normals_)
  {
    std::cout << "Plotting normals for different camera sources\n";
    plotter_->plotNormals(cloud);
  }

  // 1. Generate grasp candidates.
  double t0_candidates = omp_get_wtime();
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list =
    candidates_generator_->generateGraspCandidateSets(cloud);
  printf("Generated %zu hand sets.\n", hand_set_list.size());
  if (hand_set_list.size() == 0)
  {
    return hands_out;
  }
  double t_candidates = omp_get_wtime() - t0_candidates;
  if (plot_candidates_)
  {
    plotter_->plotFingers3D(hand_set_list, cloud.getCloudOriginal(),
      "Grasp candidates", hand_geom);
  }

  // 2. Filter the candidates.
  double t0_filter = omp_get_wtime();
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_filtered =
    filterGraspsWorkspace(hand_set_list, workspace_grasps_);
  if (hand_set_list_filtered.size() == 0)
  {
    return hands_out;
  }
  if (plot_filtered_candidates_)
  {
    plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
      "Filtered Grasps (Aperture, Workspace)", hand_geom);
  }
  if (filter_approach_direction_)
  {
    hand_set_list_filtered =
      filterGraspsDirection(hand_set_list_filtered, direction_, thresh_rad_);
    if (plot_filtered_candidates_)
    {
      plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
        "Filtered Grasps (Approach)", hand_geom);
    }
  }
  double t_filter = omp_get_wtime() - t0_filter;
  if (hand_set_list_filtered.size() == 0)
  {
    return hands_out;
  }

  // 3. Create grasp descriptors (images).
  double t0_images = omp_get_wtime();
  std::vector<std::unique_ptr<candidate::Hand>> hands;
  std::vector<std::unique_ptr<cv::Mat>> images;
  image_generator_->createImages(cloud, hand_set_list_filtered, images, hands);
  double t_images = omp_get_wtime() - t0_images;

  // 4. Classify the grasp candidates.
  double t0_classify = omp_get_wtime();
  std::vector<float> scores = classifier_->classifyImages(images);
  for (int i = 0; i < hands.size(); i++)
  {
    hands[i]->setScore(scores[i]);
  }
  double t_classify = omp_get_wtime() - t0_classify;

  // 5. Select the <num_selected> highest scoring grasps.
  hands = selectGrasps(hands);
  if (plot_valid_grasps_)
  {
    plotter_->plotFingers3D(hands, cloud.getCloudOriginal(), "Valid Grasps",
      hand_geom);
  }

  // 6. Cluster the grasps.
  double t0_cluster = omp_get_wtime();
  std::vector<std::unique_ptr<candidate::Hand>> clusters;
  if (cluster_grasps_)
  {
    clusters = clustering_->findClusters(hands);
    printf("Found %d clusters.\n", (int)clusters.size());
    if (clusters.size() <= 3)
    {
      printf(
        "Not enough clusters found! Adding all grasps from previous step.");
      for (int i = 0; i < hands.size(); i++)
      {
        clusters.push_back(std::move(hands[i]));
      }
    }
    if (plot_clustered_grasps_)
    {
      plotter_->plotFingers3D(clusters, cloud.getCloudOriginal(),
        "Clustered Grasps", hand_geom);
    }
  }
  else
  {
    clusters = std::move(hands);
  }
  double t_cluster = omp_get_wtime() - t0_cluster;

  // 7. Sort grasps by their score.
  std::sort(clusters.begin(), clusters.end(), isScoreGreater);
  printf("======== Selected grasps ========\n");
  for (int i = 0; i < clusters.size(); i++)
  {
    std::cout << "Grasp " << i << ": " << clusters[i]->getScore() << "\n";
  }
  printf("Selected the %d best grasps.\n", (int)clusters.size());
  double t_total = omp_get_wtime() - t0_total;

  printf("======== RUNTIMES ========\n");
  printf(" 1. Candidate generation: %3.4fs\n", t_candidates);
  printf(" 2. Descriptor extraction: %3.4fs\n", t_images);
  printf(" 3. Classification: %3.4fs\n", t_classify);
  // printf(" Filtering: %3.4fs\n", t_filter);
  // printf(" Clustering: %3.4fs\n", t_cluster);
  printf("==========\n");
  printf(" TOTAL: %3.4fs\n", t_total);

  if (plot_selected_grasps_)
  {
    plotter_->plotFingers3D(clusters, cloud.getCloudOriginal(),
      "Selected Grasps", hand_geom, false);
  }

  return clusters;
}

Eigen::Vector3d GraspDetector::OptToBaseFrame(const Eigen::Vector3d& in) const
{
  Eigen::Quaterniond cam_opt_q(cam_opt_rot_[0], cam_opt_rot_[1], cam_opt_rot_[2], cam_opt_rot_[3]);
  Eigen::Quaterniond base_cam_q(base_cam_rot_[0], base_cam_rot_[1], base_cam_rot_[2], base_cam_rot_[3]);
  return base_cam_q * cam_opt_q * in;
}

Eigen::Vector3d GraspDetector::OptToBaseFrame(const std::vector<double>& in) const
{
  Eigen::Vector3d vec;
  vec << in[0], in[1], in[2];
  return OptToBaseFrame(vec);
}

Eigen::Vector3d GraspDetector::BaseToOptFrame(const Eigen::Vector3d& in) const
{
  Eigen::Quaterniond cam_opt_q(cam_opt_rot_[0], cam_opt_rot_[1], cam_opt_rot_[2], cam_opt_rot_[3]);
  Eigen::Quaterniond base_cam_q(base_cam_rot_[0], base_cam_rot_[1], base_cam_rot_[2], base_cam_rot_[3]);
  return cam_opt_q.inverse() * base_cam_q.inverse() * in;
}

Eigen::Vector3d GraspDetector::BaseToOptFrame(const std::vector<double>& in) const
{
  Eigen::Vector3d vec;
  vec << in[0], in[1], in[2];
  return BaseToOptFrame(vec);
}

void GraspDetector::preprocessPointCloud(util::Cloud& cloud, const Eigen::Isometry3d& transform_base_opt)
{
  candidates_generator_->preprocessPointCloud(cloud, transform_base_opt);
}

std::vector<std::unique_ptr<candidate::HandSet>> GraspDetector::filterGraspsWorkspace(
  std::vector<std::unique_ptr<candidate::HandSet>>& hand_set_list,
  const std::vector<double>& workspace) const
{
  int remaining = 0;
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_out;
  printf("Filtering grasps outside of workspace ...\n");

  const candidate::HandGeometry& hand_geometry =
    candidates_generator_->getHandSearchParams().hand_geometry_;

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<std::unique_ptr<candidate::Hand>>& hands =
      hand_set_list[i]->getHands();
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid =
      hand_set_list[i]->getIsValid();

    for (int j = 0; j < hands.size(); j++)
    {
      if (!is_valid(j))
      {
        continue;
      }
      double half_width = 0.5 * hand_geometry.outer_diameter_;
      Eigen::Vector3d left_bottom =
        hands[j]->getPosition() + half_width * hands[j]->getBinormal();
      Eigen::Vector3d right_bottom =
        hands[j]->getPosition() - half_width * hands[j]->getBinormal();
      Eigen::Vector3d left_top =
        left_bottom + hand_geometry.depth_ * hands[j]->getApproach();
      Eigen::Vector3d right_top =
        left_bottom + hand_geometry.depth_ * hands[j]->getApproach();
      Eigen::Vector3d approach =
        hands[j]->getPosition() - 0.05 * hands[j]->getApproach();

      // Transform to base frame
      left_bottom = OptToBaseFrame(left_bottom) + base_cam_trans_;
      right_bottom = OptToBaseFrame(right_bottom) + base_cam_trans_;
      left_top = OptToBaseFrame(left_top) + base_cam_trans_;
      right_top = OptToBaseFrame(right_top) + base_cam_trans_;
      approach = OptToBaseFrame(approach) + base_cam_trans_;
      Eigen::VectorXd x(5), y(5), z(5);
      x << left_bottom(0), right_bottom(0), left_top(0), right_top(0),
        approach(0);
      y << left_bottom(1), right_bottom(1), left_top(1), right_top(1),
        approach(1);
      z << left_bottom(2), right_bottom(2), left_top(2), right_top(2),
        approach(2);

      // Ensure the object fits into the hand and avoid grasps outside the
      // workspace.
      if (hands[j]->getGraspWidth() >= min_aperture_ &&
        hands[j]->getGraspWidth() <= max_aperture_ &&
        x.minCoeff() >= workspace[0] && x.maxCoeff() <= workspace[1] &&
        y.minCoeff() >= workspace[2] && y.maxCoeff() <= workspace[3] &&
        z.minCoeff() >= workspace[4] && z.maxCoeff() <= workspace[5])
      {
        is_valid(j) = true;
        remaining++;
      }
      else
      {
        is_valid(j) = false;
      }
    }

    if (is_valid.any())
    {
      hand_set_list_out.push_back(std::move(hand_set_list[i]));
      hand_set_list_out[hand_set_list_out.size() - 1]->setIsValid(is_valid);
    }
  }

  printf("Number of grasp candidates within workspace and gripper width: %d\n",
    remaining);

  return hand_set_list_out;
}

std::vector<std::unique_ptr<candidate::HandSet>>
GraspDetector::generateGraspCandidates(const util::Cloud& cloud)
{
  return candidates_generator_->generateGraspCandidateSets(cloud);
}

std::vector<std::unique_ptr<candidate::Hand>> GraspDetector::selectGrasps(
  std::vector<std::unique_ptr<candidate::Hand>>& hands) const
{
  printf("Selecting the %d highest scoring grasps ...\n", num_selected_);

  int middle = std::min((int)hands.size(), num_selected_);
  std::partial_sort(hands.begin(), hands.begin() + middle, hands.end(),
    isScoreGreater);
  std::vector<std::unique_ptr<candidate::Hand>> hands_out;

  for (int i = 0; i < middle; i++)
  {
    hands_out.push_back(std::move(hands[i]));
    printf(" grasp #%d, score: %3.4f\n", i, hands_out[i]->getScore());
  }

  return hands_out;
}

std::vector<std::unique_ptr<candidate::HandSet>>
GraspDetector::filterGraspsDirection(
  std::vector<std::unique_ptr<candidate::HandSet>>& hand_set_list,
  const Eigen::Vector3d& direction, const double thresh_rad)
{
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_out;
  int remaining = 0;

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<std::unique_ptr<candidate::Hand>>& hands =
      hand_set_list[i]->getHands();
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid =
      hand_set_list[i]->getIsValid();

    for (int j = 0; j < hands.size(); j++)
    {
      if (is_valid(j))
      {
        double angle = acos(direction.transpose() * hands[j]->getApproach());
        if (angle > thresh_rad)
        {
          is_valid(j) = false;
        }
        else
        {
          remaining++;
        }
      }
    }

    if (is_valid.any())
    {
      hand_set_list_out.push_back(std::move(hand_set_list[i]));
      hand_set_list_out[hand_set_list_out.size() - 1]->setIsValid(is_valid);
    }
  }

  printf("Number of grasp candidates with correct approach direction: %d\n",
    remaining);

  return hand_set_list_out;
}

bool GraspDetector::createGraspImages(
  util::Cloud& cloud,
  std::vector<std::unique_ptr<candidate::Hand>>& hands_out,
  std::vector<std::unique_ptr<cv::Mat>>& images_out)
{
  // Check if the point cloud is empty.
  if (cloud.getCloudOriginal()->size() == 0)
  {
    printf("ERROR: Point cloud is empty!");
    hands_out.resize(0);
    images_out.resize(0);
    return false;
  }

  // Plot samples/indices.
  if (plot_samples_)
  {
    if (cloud.getSamples().cols() > 0)
    {
      plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
    }
    else if (cloud.getSampleIndices().size() > 0)
    {
      plotter_->plotSamples(cloud.getSampleIndices(),
        cloud.getCloudProcessed());
    }
  }

  if (plot_normals_)
  {
    std::cout << "Plotting normals for different camera sources\n";
    plotter_->plotNormals(cloud);
  }

  // 1. Generate grasp candidates.
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list =
    candidates_generator_->generateGraspCandidateSets(cloud);
  printf("Generated %zu hand sets.\n", hand_set_list.size());
  if (hand_set_list.size() == 0)
  {
    hands_out.resize(0);
    images_out.resize(0);
    return false;
  }

  const candidate::HandGeometry& hand_geom =
    candidates_generator_->getHandSearchParams().hand_geometry_;

  // 2. Filter the candidates.
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_filtered =
    filterGraspsWorkspace(hand_set_list, workspace_grasps_);
  if (plot_filtered_candidates_)
  {
    plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
      "Filtered Grasps (Aperture, Workspace)", hand_geom);
  }
  if (filter_approach_direction_)
  {
    hand_set_list_filtered =
      filterGraspsDirection(hand_set_list_filtered, direction_, thresh_rad_);
    if (plot_filtered_candidates_)
    {
      plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
        "Filtered Grasps (Approach)", hand_geom);
    }
  }

  // 3. Create grasp descriptors (images).
  std::vector<std::unique_ptr<candidate::Hand>> hands;
  std::vector<std::unique_ptr<cv::Mat>> images;
  image_generator_->createImages(cloud, hand_set_list_filtered, images_out,
    hands_out);

  return true;
}

std::vector<int> GraspDetector::evalGroundTruth(
  const util::Cloud& cloud_gt,
  std::vector<std::unique_ptr<candidate::Hand>>& hands)
{
  return candidates_generator_->reevaluateHypotheses(cloud_gt, hands);
}

std::vector<std::unique_ptr<candidate::Hand>>
GraspDetector::pruneGraspCandidates(
  const util::Cloud& cloud,
  const std::vector<std::unique_ptr<candidate::HandSet>>& hand_set_list,
  double min_score)
{
  // 1. Create grasp descriptors (images).
  std::vector<std::unique_ptr<candidate::Hand>> hands;
  std::vector<std::unique_ptr<cv::Mat>> images;
  image_generator_->createImages(cloud, hand_set_list, images, hands);

  // 2. Classify the grasp candidates.
  std::vector<float> scores = classifier_->classifyImages(images);
  std::vector<std::unique_ptr<candidate::Hand>> hands_out;

  // 3. Only keep grasps with a score larger than <min_score>.
  for (int i = 0; i < hands.size(); i++)
  {
    if (scores[i] > min_score)
    {
      hands[i]->setScore(scores[i]);
      hands_out.push_back(std::move(hands[i]));
    }
  }

  return hands_out;
}

void GraspDetector::printStdVector(const std::vector<int>& v,
  const std::string& name) const
{
  printf("%s: ", name.c_str());
  for (int i = 0; i < v.size(); i++)
  {
    printf("%d ", v[i]);
  }
  printf("\n");
}

void GraspDetector::printStdVector(const std::vector<double>& v,
  const std::string& name) const
{
  printf("%s: ", name.c_str());
  for (int i = 0; i < v.size(); i++)
  {
    printf("%3.2f ", v[i]);
  }
  printf("\n");
}

} // namespace gpd

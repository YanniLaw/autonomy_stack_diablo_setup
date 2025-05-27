#include "icp_localizer.h"

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
#if USE_MY_OWN
    m_ndt_omp.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>());
    m_ndt_omp->setResolution(0.5);
    m_ndt_omp->setStepSize(0.1);
    m_ndt_omp->setTransformationEpsilon(0.01);
    m_ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    m_ndt_omp->setNumThreads(1);
    m_ndt_omp->setMaximumIterations(50);

    m_icp_omp.reset(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
    m_icp_omp->setMaxCorrespondenceDistance(2.0);
    m_icp_omp->setTransformationEpsilon(1e-4);
    m_icp_omp->setMaximumIterations(64);
    m_icp_omp->setUseReciprocalCorrespondences(false);
#endif
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    std::cout << "Map loading..." << std::endl;
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    std::cout << "Load map success!" << std::endl;
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0)
        return false;
#if USE_MY_OWN
    m_ndt_omp->setInputSource(m_refine_inp);
    m_ndt_omp->setInputTarget(m_refine_tgt);
    m_ndt_omp->align(*aligned_cloud, guess);
    // std::cout << "NDT converged? " << m_ndt_omp->hasConverged() << " , fitness score: " << m_ndt_omp->getFitnessScore() << std::endl;
    if (!m_ndt_omp->hasConverged() || m_ndt_omp->getFitnessScore() > m_config.refine_score_thresh) {
        std::cout << "NDT not converged, fitness score " << m_ndt_omp->getFitnessScore() << std::endl;
        return false;
    }
    guess = m_ndt_omp->getFinalTransformation();

    // m_icp_omp->setInputSource(m_refine_inp);
    // m_icp_omp->setInputTarget(m_refine_tgt);
    // m_icp_omp->align(*aligned_cloud, m_ndt_omp->getFinalTransformation());
    // std::cout << "ICP converged? " << m_icp_omp->hasConverged() << " , fitness score: " << m_icp_omp->getFitnessScore() << std::endl;
    // if (!m_icp_omp->hasConverged() || m_icp_omp->getFitnessScore() > m_config.refine_score_thresh)
    //     return false;
    // guess = m_icp_omp->getFinalTransformation();
#else
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    // std::cout << "Rough ICP converged? " << m_rough_icp.hasConverged() << " , fitness score: " << m_rough_icp.getFitnessScore() << std::endl;
    if (!m_rough_icp.hasConverged() || m_rough_icp.getFitnessScore() > m_config.rough_score_thresh) {
        std::cout << "Rough ICP not converged, fitness score: " << m_rough_icp.getFitnessScore() << std::endl;
        return false;
    }
    guess = m_rough_icp.getFinalTransformation();

    // m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    // m_refine_icp.setInputSource(m_refine_inp);
    // m_refine_icp.setInputTarget(m_refine_tgt);
    // m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    // std::cout << "Refine ICP converged? " << m_refine_icp.hasConverged() << " , fitness score: " << m_refine_icp.getFitnessScore() << std::endl;
    // if (!m_refine_icp.hasConverged() || m_refine_icp.getFitnessScore() > m_config.refine_score_thresh)
    //     return false;
    // guess = m_refine_icp.getFinalTransformation();
#endif
    return true;
}
#include "MyVoronoi.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <opencv2/core/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <streambuf>


#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

#include "Random.h"
void drawCross(cv::Mat img, cv::Point point, int size, cv::Scalar color, int thickness);

namespace boost
{
    namespace polygon
    {
        template <>
        struct geometry_concept<cv::Point2i>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<cv::Point2i>
        {
            typedef double coordinate_type;

            static inline coordinate_type get(const cv::Point2i& point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };
    }  // polygon
}  // boost

MyVoronoi::MyVoronoi(int n_points, int rows, int cols, int seed, bool regularize)
{
    SetSites(n_points, rows, cols, seed, regularize);
    m_vmap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    m_vmap.setTo(255);
    
}

MyVoronoi::~MyVoronoi()
{
    //Finalize();
}

int MyVoronoi::nsites() { return m_sites.size(); }

void MyVoronoi::SaperateDensityMap(GenDensityMap* DensityMap, float GlobalScellDensity)
{
    m_DensityMap = DensityMap;
    cv::Mat outmap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    outmap.setTo(255);
    
    m_sites.clear();

    
    std::map<std::pair<int, int>, int> PointMap;
    std::vector<SubVD> First_to_Second_SubVor;
    for (int i = 0; i < m_DensityMap->ReturnVoronoiDiagram()->cells().size(); i++) //iterate all the cells
    {
        int cell_site_index = m_DensityMap->ReturnVoronoiDiagram()->cells()[i].source_index();
        
        double k = m_DensityMap->ReturnAlphaList()->at(cell_site_index) / 255.0;
        int n_points;
        int Contrast = 50;  //The vertical intercept of the decrease line
        n_points = Contrast - Contrast * k; //n_point <=> y ~[0, Constrast]; x ~[0, 1]; => k
        n_points *= GlobalScellDensity;//porcelin:1//0.5;//pitch,mud:3;//cement:5
        SubVD sub_voronoi(n_points, m_DensityMap, i, &PointMap, true);

        std::cout << "Cell " << i << "Deviding..." << std::endl;
        First_to_Second_SubVor.push_back(sub_voronoi);
    
        for (int p = 0; p < sub_voronoi.ReturnSiteSet().size(); ++p)
        {
            int m = sub_voronoi.ReturnSiteSet()[p].x;
            int n = sub_voronoi.ReturnSiteSet()[p].y;

            cv::circle(outmap, Site(m, n), 4, cv::Scalar(0, 0, 255), -1);
        }
    }

    m_SubdevideHierarchy.push_back(First_to_Second_SubVor);  //for every level, the number of elements == the number of the cells in the parent global vornoi map
    
    std::ostringstream os1;
    os1 << "./DensityMap/" << "Density_Sites" << ".jpg";
    cv::imwrite(os1.str(), outmap);
}

void MyVoronoi::SaperateSubVorMap(voronoi_diagram<double>* vd, std::vector<cv::Point2i>* sites, int Density)
{
    // for subdevide sub voronoi map. Please follow the function "SaperateDensityMap()"
}

std::vector<std::pair<std::vector<cv::Point2i>, std::vector<cv::Point3f>>> MyVoronoi::DrawTopo()
{
    std::vector<std::pair<std::vector<cv::Point2i>, std::vector<cv::Point3f>>> output;
    cv::Point3f C0 = cv::Point3f(1, 0, 0);
    cv::Point3f C1 = cv::Point3f(1, 1, 0);
    cv::Point3f C2 = cv::Point3f(0, 1, 0);
    cv::Point3f C3 = cv::Point3f(0, 1, 1);

    std::vector<cv::Point2i> point_garage0;
    std::vector<cv::Point3f> point_color_garage0;
    //for pitch, need to add


    std::vector<cv::Point2i> point_garage1;     //big cell original
    std::vector<cv::Point3f> point_color_garage1;


    for (auto edge : m_Voronoi_Hierarchy.m_elist[0])
    {
        Site p0, p1;

        if (edge->m_verts[0] == NULL)
        {
            p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
            p0 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
        }
        else if (edge->m_verts[1] == NULL)
        {
            p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
            p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
        }
        else
        {
            p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
            p1 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
        }
        point_garage1.push_back(p0);
        point_garage1.push_back(p1);
        point_color_garage1.push_back(C1);
        point_color_garage1.push_back(C1);
    }


    std::vector<cv::Point2i> point_garage2;     //big cell saparated to small
    std::vector<cv::Point3f> point_color_garage2;
    for (int i = 0; i < m_Voronoi_Hierarchy.m_elist[0].size(); i++)
    {
        for (auto edge : m_Voronoi_Hierarchy.m_elist[0][i]->m_children)
        {
            Site p0, p1;

            if (edge->m_verts[0] == NULL)
            {
                p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
                p0 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
            }
            else if (edge->m_verts[1] == NULL)
            {
                p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
                p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
            }
            else
            {
                p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
                p1 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
            }
            point_garage2.push_back(p0);
            point_garage2.push_back(p1);
            point_color_garage2.push_back(C2);
            point_color_garage2.push_back(C2);
        }
    }

    std::vector<cv::Point2i> point_garage3;     //small cell original
    std::vector<cv::Point3f> point_color_garage3;

    for (auto edge : m_Voronoi_Hierarchy.m_elist[1])
    {
        Site p0, p1;

        if (edge->m_verts[0] == NULL)
        {
            p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
            p0 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
        }
        else if (edge->m_verts[1] == NULL)
        {
            p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
            p1 = Site(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
        }
        else
        {
            p0 = Site(edge->m_verts[0]->m_pos.x, edge->m_verts[0]->m_pos.y);
            p1 = Site(edge->m_verts[1]->m_pos.x, edge->m_verts[1]->m_pos.y);
        }
        point_garage3.push_back(p0);
        point_garage3.push_back(p1);
        point_color_garage3.push_back(C3);
        point_color_garage3.push_back(C3);
    }

    output.push_back({ point_garage0, point_color_garage0 });
    output.push_back({ point_garage1, point_color_garage1 });
    output.push_back({ point_garage2, point_color_garage2 });
    output.push_back({ point_garage3, point_color_garage3 });

    return output;
}

void MyVoronoi::ConstructTopo()
{
    std::vector<voronoi_diagram<double>*> vd_layers;
    std::vector<std::vector<Site>*> sites_layers;
    std::vector<std::vector<int>> site_arrangement;  //for parent and kids relationship  [26,40,55...] <=> [0,26) [26,40) [40,55)
    std::vector<std::vector<double>> site_alphas_garage;
    voronoi_diagram<double> subvds[2];

    //(for test)std::map<std::pair<int, int>, int> hash;

    for (int i = 0; i < m_level; i++)
    {
        if (i == 0)
        {
            std::vector<double> site_alphas;

            vd_layers.push_back(m_DensityMap->ReturnVoronoiDiagram());
            sites_layers.push_back(m_DensityMap->ReturnSiteSet());

            for (int i = 0; i < m_DensityMap->ReturnSiteSet()->size(); i++)
            {
                site_alphas.push_back(m_DensityMap->ReturnAlphaList()->at(i) / 255.0 * 1.13 * 0.01);
            }
            site_alphas_garage.push_back(site_alphas);
        }
        else
        {   
            std::vector<Site> site;
            std::vector<int> site_array;
            std::vector<double> site_alphas;

            //subindex = 0 => m_SubdevideHierarchy[i-1].size() == density map cell's number 
            for (int subindex = 0; subindex < m_SubdevideHierarchy[i - 1].size(); subindex++)  //every big cell
            {
                std::vector<Site> s = m_SubdevideHierarchy[i - 1][subindex].ReturnSiteSet();  //small cell sites of every subvd 
                for (int j = 0; j < s.size(); j++)
                {
                    site.push_back(s[j]);
                    //hash[{s[j].x, s[j].y}] += 1;
                    double t = m_SubdevideHierarchy[i - 1][subindex].ReturnAlphaValue();
                    site_alphas.push_back(m_SubdevideHierarchy[i - 1][subindex].ReturnAlphaValue());
                }
                site_array.push_back(s.size());
            }

            //std::vector< std::pair<int, int>> k;
            //for (auto it = hash.begin(); it != hash.end(); it++)
            //{
            //    if (it->second > 1)
            //    {
            //        k .push_back(it->first);
            //    }
            //}

            m_SiteGarage.push_back(site);
            if (i == 1)
            {
                m_vd.clear();
                construct_voronoi(m_SiteGarage.back().begin(), m_SiteGarage.back().end(), &m_vd);
                vd_layers.push_back(&m_vd);
            }
            else
            {
                construct_voronoi(m_SiteGarage.back().begin(), m_SiteGarage.back().end(), &subvds[i - 1]);
                vd_layers.push_back(&subvds[i - 1]);
            }
            
            sites_layers.push_back(&(m_SiteGarage.back()));
            site_arrangement.push_back(site_array);
            site_alphas_garage.push_back(site_alphas);
        }
    }

    m_Voronoi_Hierarchy.Initilization(vd_layers, sites_layers, site_arrangement, site_alphas_garage);

    m_sites.clear();
    m_sites = m_SiteGarage[0];
    m_sites_alpha.clear();
    m_sites_alpha = (site_alphas_garage.front());

}

//double cosBetween(const cv::Point2d& v1, const cv::Point2d& v2)
//{
//    double cosAngle = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
//    return cosAngle;
//}

cv::Point2d FindFoot(cv::Point2d pntSart, cv::Point2d pntEnd, cv::Point2d pA)
{
    cv::Point2d pFoot;
    double k = 0.0;
    if (pntSart.x == pntEnd.x)
    {
        pFoot.x = pntSart.x;
        pFoot.y = pA.y;
        return pFoot;
    }
    k = (pntEnd.y - pntSart.y) * 1.0 / (pntEnd.x - pntSart.x);
    double A = k;
    double B = -1.0;
    double C = pntSart.y - k * pntSart.x;

    pFoot.x = (B * B * pA.x - A * B * pA.y - A * C) / (A * A + B * B);
    pFoot.y = (A * A * pA.y - A * B * pA.x - B * C) / (A * A + B * B);

    return pFoot;
}

void LineFitLeastSquares(std::vector<vec2> data, cv::Point2d & pp0, cv::Point2d& pp1)
{
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;

    for (int i = 0; i < data.size(); i++)
    {
        A += data[i].x * data[i].x;
        B += data[i].x;
        C += -data[i].x * data[i].y;
        D += -data[i].y;
    }

    double k, b, temp = 0.0;
    if (temp = (data.size()* A - B * B)) // not 0 for deviding
    {
        k = (data.size() * C - B * D) / temp;
        b = (A * D - B * C) / temp;
    }
    else
    {
        k = 1;
        b = 0;
    }
    pp0.x = 0; pp0.y = -(k * pp0.x + b); 
    pp1.x = 1000; pp1.y = -(k * pp1.x + b);
}

void MyVoronoi::SmoothHVDedges(double Theta) //Theta is the value percentage change of the angle
{
    Theta = 1.0 - Theta;
    cv::Mat k = cv::Mat::zeros(1000, 1000, CV_8UC3); //color_jet CV_32FC3
    k.setTo(255);
    for (auto pe : m_Voronoi_Hierarchy.m_elist[0])
    {
        //WAY TWO: find the least square line, merge the points onto the LS line
        //std::unordered_map<int, int> ifseevertbefore;
        //std::vector<vec2> vertexdata;
        //for (int i = 0; i < pe->m_children.size(); i++)
        //{
        //    HVD::edge* edge = pe->m_children[i];
        //    if (edge->m_verts[0] != NULL)
        //    {    
        //        if (ifseevertbefore.find(edge->m_verts[0]->m_index) == ifseevertbefore.end())
        //        {
        //            vertexdata.push_back(edge->m_verts[0]->m_oldpos);
        //            ifseevertbefore[edge->m_verts[0]->m_index] = 1;
        //        }
        //    }
        //    else
        //    {
        //        vertexdata.push_back(edge->m_infinit_vert);
        //    }
        //
        //    if (edge->m_verts[1] != NULL) 
        //    {
        //        if (ifseevertbefore.find(edge->m_verts[1]->m_index) == ifseevertbefore.end())
        //        {
        //            vertexdata.push_back(edge->m_verts[1]->m_oldpos);
        //            ifseevertbefore[edge->m_verts[1]->m_index] = 1;
        //        }
        //    }
        //    else
        //    {
        //        vertexdata.push_back(edge->m_infinit_vert);
        //    }
        //}
        //cv::Point2d pp0, pp1;
        //LineFitLeastSquares(vertexdata, pp0, pp1);

        //WAY ONE: directly merged onto parent edge
        cv::Point2d pp0, pp1;
        if (pe->m_verts[0] == NULL)
        {
            pp0 = cv::Point2d(pe->m_infinit_vert.x, pe->m_infinit_vert.y);
            pp1 = cv::Point2d(pe->m_verts[1]->m_pos.x, pe->m_verts[1]->m_pos.y);
        }
        else if (pe->m_verts[1] == NULL)
        {
            pp0 = cv::Point2d(pe->m_verts[0]->m_pos.x, pe->m_verts[0]->m_pos.y);
            pp1 = cv::Point2d(pe->m_infinit_vert.x, pe->m_infinit_vert.y);
        }
        else
        {
            pp0 = cv::Point2d(pe->m_verts[0]->m_pos.x, pe->m_verts[0]->m_pos.y);
            pp1 = cv::Point2d(pe->m_verts[1]->m_pos.x, pe->m_verts[1]->m_pos.y);
        }
        

        //move toward to the least square line
        for (int i = 0; i < pe->m_children.size(); i++)
        {

            cv::Point2d p0, p1;
            HVD::edge* edge = pe->m_children[i];
            if (edge->m_verts[0] == NULL)
            {
                p0 = cv::Point2d(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
                p1 = cv::Point2d(edge->m_verts[1]->m_oldpos.x, edge->m_verts[1]->m_oldpos.y);
            }
            else if (edge->m_verts[1] == NULL)
            {
                p0 = cv::Point2d(edge->m_verts[0]->m_oldpos.x, edge->m_verts[0]->m_oldpos.y);
                p1 = cv::Point2d(edge->m_infinit_vert.x, edge->m_infinit_vert.y);
            }
            else
            {
                p0 = cv::Point2d(edge->m_verts[0]->m_oldpos.x, edge->m_verts[0]->m_oldpos.y);
                p1 = cv::Point2d(edge->m_verts[1]->m_oldpos.x, edge->m_verts[1]->m_oldpos.y);
            }

            cv::Point2d p0Foot = FindFoot(pp0, pp1, p0);
            cv::Point2d p1Foot = FindFoot(pp0, pp1, p1);
            cv::Point2d Foot_to_p0 = p0 - p0Foot;
            cv::Point2d Foot_to_p1 = p1 - p1Foot;
            cv::line(k, p0, p0Foot, cv::Scalar(255, 0, 0), 1);
            cv::line(k, p1, p1Foot, cv::Scalar(255, 0, 0), 1);

            if (i == 0)
            {
                if (edge->m_verts[0] != NULL)
                {
                    cv::Point2d p0_now = p0Foot + Foot_to_p0 * Theta;
                    edge->m_verts[0]->m_pos.x = p0_now.x;
                    edge->m_verts[0]->m_pos.y = p0_now.y;
                }
                if (edge->m_verts[1] != NULL)
                {
                    cv::Point2d p1_now = p1Foot + Foot_to_p1 * Theta;
                    edge->m_verts[1]->m_pos.x = p1_now.x;
                    edge->m_verts[1]->m_pos.y = p1_now.y;
                }
            }
            else
            {
                if (edge->m_verts[0] == pe->m_children[i - 1]->m_verts[0] || edge->m_verts[0] == pe->m_children[i - 1]->m_verts[1])
                {
                    if (edge->m_verts[1] != NULL)
                    {
                        cv::Point2d p1_now = p1Foot + Foot_to_p1 * Theta;
                        edge->m_verts[1]->m_pos.x = p1_now.x;
                        edge->m_verts[1]->m_pos.y = p1_now.y;
                    }
                }
                else
                {
                    if (edge->m_verts[0] != NULL)
                    {
                        cv::Point2d p0_now = p0Foot + Foot_to_p0 * Theta;
                        edge->m_verts[0]->m_pos.x = p0_now.x;
                        edge->m_verts[0]->m_pos.y = p0_now.y;
                    }
                }

            }


        }
    }
    cv::imwrite("./CrackMaps/direction.png", k);
}

void MyVoronoi::SetSites(int n_points, int rows, int cols, int seed, bool regularize)  // For the initial default sites' loading
{
    m_row = rows;
    m_column = cols;
    m_num_points = n_points;
    //m_Temp_Grid = NULL;
    m_FDE_Mesh = NULL;
    std::uniform_int_distribution<int> dist(0, rows * cols - 1); //get random int between [0, rows*cols-1]
    //std::poisson_distribution<> d(500);
    std::exponential_distribution<> d(0.0035);
    seed_rand(seed);

    if (regularize)
    {
        cv::Mat occupied(rows, cols, CV_8UC1, cv::Scalar(0));
        int n_x = sqrt(n_points * cols / rows);
        int max_dist = 0.8 * cols / (n_x + 1); // Heuristic for maximum peak distance 
        
        //----square grid----//
        //int col_nums = sqrt(n_points);
        //int dist = cols / (col_nums);
        int iterate_limit = 0;
        for (int i = 0; i < n_points; ++i)
        {
            int x, y;
            // Generate points until one is accepted
            do {
                int index = dist(pseudo_rand_engine);
                x = index % cols;                        //O--------->X
                y = index / cols;   
                iterate_limit += 1; //|
            } while (occupied.at<uchar>(y, x) && iterate_limit < 30);          //|
                                                         //|
            m_sites.push_back(Site(x, y));               //Y
            cv::circle(occupied, Site(x, y), max_dist, 25, cv::FILLED);
            //cv::circle(occupied, Site(x, y), max_dist, 255, cv::FILLED); !!!!!!!!NICEEEE!!!!
            
            //cv::circle(occupied, Site(x, y), max_dist, 255, 1);
            //cv::circle(occupied, Site(x, y), 1, 255, -1);
            
            //cv::circle(heightmap, v.getPoints()[i], 1, Scalar(0, 255, 0), -1);

            //----square grid----//
            ////int x = (0.5 + i % col_nums) * dist;
            ////int y = (0.5 + i / col_nums) * dist;
            ////m_sites.push_back(Site(x, y));
        }
        //cv::imwrite("Hu.png", occupied);
    }
    else
    {
        //for (int i = 0; i < n_points; ++i)
        //{
        //    int index = dist(pseudo_rand_engine);
        //    int x = index % cols;
        //    int y = index / cols;
        //    m_sites.push_back(Site(x, y));
        //    //cv::circle(heightmap, v.getPoints()[i], 1, Scalar(0, 255, 0), -1);
        //}
        int indexy = -1;
        for (int i = 0; i < n_points; ++i)
        {
            do { indexy = d(pseudo_rand_engine); } while (indexy < 0 || indexy >= rows);
            int x = Random::Float() * (cols - 1);  //indexx;
            int y = (rows-1) - indexy;
            m_sites.push_back(Site(x, y));
        }
    }

}

void MyVoronoi::Construct_VoronoiMap()
{
    if (m_vd.num_cells() != 0) m_vd.clear();
    construct_voronoi(m_sites.begin(), m_sites.end(), &m_vd);

    if (m_Voronoi_Damage_Diagram.num_cells() != 0) m_Voronoi_Damage_Diagram.clear();
    construct_voronoi(m_sites.begin(), m_sites.end(), &m_Voronoi_Damage_Diagram);
}

void MyVoronoi::InsertDamagetoVoronoi()  // to initialize the voronoi diagram with damage points
{
    m_sites_with_Damage = m_sites;
    int dis = 10;  // the radius of the damage core (should be the even number)
    for (auto p : m_Damage_Points)
    {
        cv::Point q = cv::Point(p.x, p.y);
        m_sites_with_Damage.push_back(Site(p.x, p.y));
        
        int k = 0;
        Site jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x, p.y + dis) + jitter);
        jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x - dis, p.y + dis/2) + jitter);
        jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x - dis, p.y - dis/2) + jitter);
        jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x, p.y - dis) + jitter);
        jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x + dis, p.y - dis / 2) + jitter);
        jitter = Site(k * (Random::Float() - 0.5), k * (Random::Float() - 0.5));
        m_sites_with_Damage.push_back(Site(p.x + dis, p.y + dis / 2) + jitter);
    }
    m_Voronoi_Damage_Diagram.clear();
    construct_voronoi(m_sites_with_Damage.begin(), m_sites_with_Damage.end(), &m_Voronoi_Damage_Diagram);
    

}

void MyVoronoi::SetDamagePoint(vec3 P)
{
    m_Damage_Points.push_back(P);
}

vec2 MyVoronoi::ClearLastDamagePoint()
{
    vec2 p = vec2(m_Damage_Points.back().x, m_Damage_Points.back().y);
    m_Damage_Points.pop_back();
    return p;
}

void MyVoronoi::ClearAllDamagePoint()
{
     m_Damage_Points.erase(m_Damage_Points.begin(), m_Damage_Points.end());
}


void MyVoronoi::ShowDamage()
{
    cv::Mat outmap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    outmap.setTo(255);

    cv::Mat outholes = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    outholes.setTo(255);

    for (auto i : m_Damage_Points)
    {
        //damage points part
        //GradientCircle2(outholes, Site(i.x + (int)(7 * (Random::Float() - 0.5)), i.y + (int)(7 * (Random::Float() - 0.5))), 13);
        cv::circle(outholes, Site(i.x, i.y), 8, cv::Scalar(0, 0, 0), -1);
    }
    cv::blur(outholes, outholes, cv::Size(5, 5));
    //for (auto i : m_Damage_Points)
    //{
    //    cv::circle(outholes, Site(i.x, i.y), 8, cv::Scalar(0, 0, 0), -1);
    //}


    m_damageseeds.erase(m_damageseeds.begin(), m_damageseeds.end());

    if (m_Damage_Points.size()!=0)
    {
        for (voronoi_diagram<double>::const_cell_iterator it = m_Voronoi_Damage_Diagram.cells().begin(); 
            it != m_Voronoi_Damage_Diagram.cells().end(); ++it)
        {
            const voronoi_diagram<double>::cell_type& cell = *it;
            if (cell.source_index() >= m_sites.size() && (cell.source_index()- m_sites.size()) % 7 == 0)
            {
                m_damageseeds.push_back(it);
                const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
                std::vector<cv::Point> elementPoints;
                if (edge->is_finite())
                {
                    if (edge->twin()->next()->is_finite())
                    {
                        elementPoints.push_back(cv::Point(edge->vertex0()->x(), edge->vertex0()->y())); // record the vertex
                        Site p1 = Site(edge->twin()->next()->vertex0()->x(), edge->twin()->next()->vertex0()->y());  //draw all the possible crack direction
                        Site p2 = Site(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                        cv::arrowedLine(outmap, p1, p2, cv::Scalar(0, 0, 0), 1);
                        Site p1d = Site(m_sites_with_Damage[cell.source_index()].x, m_sites_with_Damage[cell.source_index()].y);
                        Site p2d = Site(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                        DrawLine(p1d, p2d, outholes,3);
                    }
                }
                int s = 0;
                do {
                    edge = edge->next();
                    if (edge->is_finite())
                    {
                        if (edge->twin()->next()->is_finite())
                        {
                            if (s == 2)
                            {
                                elementPoints.push_back(cv::Point(edge->vertex0()->x(), edge->vertex0()->y())); // record the vertex
                                Site p1 = Site(m_sites_with_Damage[cell.source_index()].x, m_sites_with_Damage[cell.source_index()].y);
                                Site p2 = Site(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                                DrawLine(p1, p2, outholes,3);
                            }

                            elementPoints.push_back(cv::Point(edge->vertex0()->x(), edge->vertex0()->y())); // record the vertex
                            Site p1 = Site(edge->twin()->next()->vertex0()->x(), edge->twin()->next()->vertex0()->y());  //draw all the possible crack direction
                            Site p2 = Site(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                            cv::arrowedLine(outmap, p1, p2, cv::Scalar(0, 0, 0), 1);
                            s++;
                        }
                    }
                } while (edge != cell.incident_edge());
                cv::fillPoly(outmap, elementPoints, cv::Scalar(0, 0, 0), 8);

            }
        }
    }

    //for (auto i : m_Damage_Points)
    //{
    //    cv::circle(outmap, Site(i.x, i.y), 3, cv::Scalar(255, 0, 0), -1);
    //}

    //GaussianBlur(outholes, outholes, cv::Size(3, 3), 0);
    
    //std::ostringstream os1;
    //os1 << "./CrackMaps/" << "Damage_Holes" << ".png";
    //cv::imwrite(os1.str(), outholes);

    std::ostringstream os2;
    os2 << "./CrackMaps/" << "Damage_Map" << ".png";
    cv::imwrite(os2.str(), outmap);

    std::cout << "Saved image as " << os2.str() << "\n";
}

void MyVoronoi::ShowDamageWithVoronoi()
{
    cv::Mat outmap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    outmap.setTo(255);

    for (int i = 0; i < m_Damage_Points.size(); i++)
    {
        //cv::circle(outmap, cv::Point(m_Damage_Points[i].x, m_Damage_Points[i].y), 3, cv::Scalar(255, 0, 0), -1);
        drawCross(outmap, cv::Point(m_Damage_Points[i].x, m_Damage_Points[i].y), 10, cv::Scalar(0, 123, 0), 1);
    }

    for (voronoi_diagram<double>::const_cell_iterator it = m_Voronoi_Damage_Diagram.cells().begin(); it != m_Voronoi_Damage_Diagram.cells().end(); ++it)
    {
        const voronoi_diagram<double>::cell_type& cell = *it;
        const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();

        
        //---------------------The original cells----------------------------------------------------
        do {
            Site p1;
            Site p2;

            if (edge->is_finite())
            {
                p1 = Site(edge->vertex0()->x(), edge->vertex0()->y());
                p2 = Site(edge->vertex1()->x(), edge->vertex1()->y());
                cv::line(outmap, p1, p2, cv::Scalar(0, 0, 255), 1);
            }
            else
            {
                // This covers the infinite edge case in question.
                const boost::polygon::voronoi_vertex<double>* v0 = edge->vertex0();
                //// Again, only consider half the half-edges, ignore edge->vertex1()
                //// to avoid overdrawing the lines
                if (v0)
                {
                    // Direction of infinite edge if perpendicular to direction
                    // between the points owning the two half edges. 
                    // Take the rotated right vector and multiply by a large 
                    // enough number to reach your bounding box

                    Site p1 = m_sites_with_Damage[edge->cell()->source_index()];
                    Site p2 = m_sites_with_Damage[edge->twin()->cell()->source_index()];
                    int end_x = (p1.y - p2.y) * 2;
                    int end_y = (p2.x - p1.x) * 2;
                    cv::circle(outmap, Site(p1.x, p1.y), 1, cv::Scalar(255, 0, 0), -1);
                    cv::circle(outmap, Site(p2.x, p2.y), 1, cv::Scalar(255, 0, 0), -1);
                    cv::line(outmap, Site(v0->x(), v0->y()), Site(v0->x() + end_x, v0->y() + end_y), cv::Scalar(0, 0, 255), 1);
                }
            }
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }

    m_damageseeds.erase(m_damageseeds.begin(), m_damageseeds.end());
    for (voronoi_diagram<double>::const_cell_iterator it = m_Voronoi_Damage_Diagram.cells().begin(); it != m_Voronoi_Damage_Diagram.cells().end(); ++it)
    {
        const voronoi_diagram<double>::cell_type& cell = *it;
        const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();

        //---------------------The new added cell----------------------------------------------------
        if (cell.source_index() >= m_sites.size() && (cell.source_index() - m_sites.size()) % 7 == 0)
        {
            m_damageseeds.push_back(it);
            const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
            std::vector<cv::Point> elementPoints;
            if (edge->is_finite())
            {
                const voronoi_diagram<double>::edge_type* twin_edge = edge->twin();
                if (twin_edge->is_finite())
                {
                    Site p10 = Site(twin_edge->vertex0()->x(), twin_edge->vertex0()->y());  //draw all the possible crack direction
                    Site p20 = Site(twin_edge->vertex1()->x(), twin_edge->vertex1()->y());
                    cv::line(outmap, p10, p20, cv::Scalar(255, 0, 0), 1);
                }
                do {
                    twin_edge = twin_edge->next();
                    if (twin_edge->is_finite())
                    {
                        Site p1 = Site(twin_edge->vertex0()->x(), twin_edge->vertex0()->y());  //draw all the possible crack direction
                        Site p2 = Site(twin_edge->vertex1()->x(), twin_edge->vertex1()->y());
                        cv::line(outmap, p1, p2, cv::Scalar(255, 0, 0), 1);
                    }
                } while (twin_edge != edge->twin());
            }
            do {
                edge = edge->next();
                if (edge->is_finite())
                {
                    const voronoi_diagram<double>::edge_type* twin_edge = edge->twin();
                    if (twin_edge->is_finite())
                    {
                        Site p10 = Site(twin_edge->vertex0()->x(), twin_edge->vertex0()->y());  //draw all the possible crack direction
                        Site p20 = Site(twin_edge->vertex1()->x(), twin_edge->vertex1()->y());
                        cv::line(outmap, p10, p20, cv::Scalar(255, 0, 0), 1);
                    }
                    do {
                        twin_edge = twin_edge->next();
                        if (twin_edge->is_finite())
                        {
                            Site p1 = Site(twin_edge->vertex0()->x(), twin_edge->vertex0()->y());  //draw all the possible crack direction
                            Site p2 = Site(twin_edge->vertex1()->x(), twin_edge->vertex1()->y());
                            cv::line(outmap, p1, p2, cv::Scalar(255, 0, 0), 1);
                        }
                    } while (twin_edge != edge->twin());
                }
            } while (edge != cell.incident_edge());
        }
    }

    DrawRuler(outmap);

    std::ostringstream os;
    os << "./CrackMaps/" << "Voronoi_Damage_Map" << ".png";
    cv::imwrite(os.str(), outmap);

    std::cout << "Saved image as " << os.str() << "\n";

    //CalDamHomogeneous();
}


int getAngelOfTwoVector(cv::Point pt1, cv::Point pt2, cv::Point c)
{
    vec2 v1 = vec2(pt1.x - c.x, pt1.y - c.y);
    vec2 v2 = vec2(pt2.x - c.x, pt2.y - c.y);
    double l1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    double l2 = sqrt(v2.x * v2.x + v2.y * v2.y);
    double err = 0.00001;
    double theta;
    if ((l1 > err) && (l2 > err))
    {
        double dotp = (v1.x * v2.x + v1.y * v2.y);
        double length = (l1 * l2);
        theta = acos(dotp / length);
    }
    theta = round(theta * 180.0 / CV_PI);
    return theta;
}

std::string unicode_to_utf8(int unicode)
{
    std::string s;

    if (unicode >= 0 && unicode <= 0x7f)  // 7F(16) = 127(10)
    {
        s = static_cast<char>(unicode);

        return s;
    }
    else if (unicode <= 0x7ff)  // 7FF(16) = 2047(10)
    {
        unsigned char c1 = 192, c2 = 128;

        for (int k = 0; k < 11; ++k)
        {
            if (k < 6)  c2 |= (unicode % 64) & (1 << k);
            else c1 |= (unicode >> 6) & (1 << (k - 6));
        }

        s = c1;    s += c2;

        return s;
    }
    else if (unicode <= 0xffff)  // FFFF(16) = 65535(10)
    {
        unsigned char c1 = 224, c2 = 128, c3 = 128;

        for (int k = 0; k < 16; ++k)
        {
            if (k < 6)  c3 |= (unicode % 64) & (1 << k);
            else if (k < 12) c2 |= (unicode >> 6) & (1 << (k - 6));
            else c1 |= (unicode >> 12) & (1 << (k - 12));
        }

        s = c1;    s += c2;    s += c3;

        return s;
    }
    else if (unicode <= 0x1fffff)  // 1FFFFF(16) = 2097151(10)
    {
        unsigned char c1 = 240, c2 = 128, c3 = 128, c4 = 128;

        for (int k = 0; k < 21; ++k)
        {
            if (k < 6)  c4 |= (unicode % 64) & (1 << k);
            else if (k < 12) c3 |= (unicode >> 6) & (1 << (k - 6));
            else if (k < 18) c2 |= (unicode >> 12) & (1 << (k - 12));
            else c1 |= (unicode >> 18) & (1 << (k - 18));
        }

        s = c1;    s += c2;    s += c3;    s += c4;

        return s;
    }
    else if (unicode <= 0x3ffffff)  // 3FFFFFF(16) = 67108863(10)
    {
        ;  // actually, there are no 5-bytes unicodes
    }
    else if (unicode <= 0x7fffffff)  // 7FFFFFFF(16) = 2147483647(10)
    {
        ;  // actually, there are no 6-bytes unicodes
    }
    else;  // incorrect unicode (< 0 or > 2147483647)

    return "";
}


//void MyVoronoi::CreateCrack()
//{
//    if (m_Voronoi_Damage_Diagram.num_cells() == 0) // when there is no damage use the original voronoi diagram
//    {
//        int sitesize = m_Voronoi_Hierarchy.m_vlist[1].size(); 
//
//        /* generate secret number between 1 and sitesize: */
//        int random_site = (sitesize - 1) * Random::Float();//rand() % sitesize + 1;
//        int new_index = random_site - 1;
//
//        // Initialize the branch
//        seed_rand(-1);
//        
//        HVD::vert* startvertex = m_Voronoi_Hierarchy.m_vlist[1][new_index];
//        if (startvertex->m_pos.x >= 0 && startvertex->m_pos.y >= 0 && startvertex->m_pos.x <= m_column && startvertex->m_pos.y <= m_row)
//        {
//            if (m_garbage_vets.find(startvertex->m_index) == m_garbage_vets.end())
//            {
//                std::poisson_distribution<> d(100); //40
//                double potential = d(pseudo_rand_engine);  // most possible growth potentical for all of the trees: 10
//                Node* startnode = new Node(0, startvertex, potential);
//                
//                m_crackseeds.push_back(startnode);
//            }
//            else qWarning() << "Created crackseed contrast. Please add a new seed again. (0 -- garbage point)";
//        }
//        else qWarning() << "Created crackseed contrast. Please add a new seed again. (1 -- invisible point)";
//        
//    }
//    else  // when there are some damage use the voronoi diagram with damage point
//    {
//       qWarning() << "HAHAHAHAHAH";
//    }
//}
void MyVoronoi::CreateCrack()
{
    if (m_Voronoi_Damage_Diagram.num_cells() == 0) // when there is no damage use the original voronoi diagram
    {
        int edgesize = m_Voronoi_Hierarchy.m_elist[0].size(); 

        /* generate secret number between 1 and sitesize: */
        int random_site = (edgesize - 1) * Random::Float();//rand() % sitesize + 1;
        int new_index = random_site - 1;

        // Initialize the branch
        seed_rand(-1);
        
        HVD::vert* startvertex;
        if (m_Voronoi_Hierarchy.m_elist[0][new_index]->m_children[0]->m_verts[0] == NULL)
            startvertex = m_Voronoi_Hierarchy.m_elist[0][new_index]->m_children[0]->m_verts[1];
        else
            startvertex = m_Voronoi_Hierarchy.m_elist[0][new_index]->m_children[0]->m_verts[0];

        if (startvertex->m_pos.x >= 0 && startvertex->m_pos.y >= 0 && startvertex->m_pos.x <= m_column && startvertex->m_pos.y <= m_row)
        {
            if (m_garbage_vets.find(startvertex->m_index) == m_garbage_vets.end())
            {
                std::poisson_distribution<> d(600); //pitch: 100 //wallpainting: 40
                double potential = d(pseudo_rand_engine);  // most possible growth potentical for all of the trees: 10
                Node* startnode = new Node(0, startvertex, potential);
                
                m_crackseeds.push_back(startnode);
            }
            else qWarning() << "Created crackseed contrast. Please add a new seed again. (0 -- garbage point)";
        }
        else qWarning() << "Created crackseed contrast. Please add a new seed again. (1 -- invisible point)";
        
    }
    else  // when there are some damage use the voronoi diagram with damage point
    {
       qWarning() << "HAHAHAHAHAH";
    }
}

void MyVoronoi::SetTempVectorField(FDE_Topology* T)
{
    //m_Temp_Grid = T; //In order to get the value from TempGrid in real time
    m_FDE_Mesh = T;
}

void MyVoronoi::SetSecondStartOnPriEdge()
{
    for (auto e : m_Voronoi_Hierarchy.m_elist[0])
    {
        if (e->m_break_flag == true && e->m_children.size()!=0)
        {
            //[CHANGE] arbitrary setting over here in the middle of big edge, 
            //later need to decide according to the potential energy on the cell
            int selectedge = e->m_children.size() / 2;
            if (e->m_children[selectedge]->m_verts[0] != NULL)
            e->m_secondarybreakpoint[e->m_cells[0]->m_index] 
                = e->m_children[selectedge]->m_verts[0]->m_index;
            if (e->m_children[selectedge]->m_verts[1] != NULL)
            e->m_secondarybreakpoint[e->m_cells[1]->m_index]
                = e->m_children[selectedge]->m_verts[1]->m_index;
        }
    }
}

void MyVoronoi::TestDrawPriandSecEdge()
{
    //add all the first level nodes
    if (m_crackseeds.size() == 0)
    {
        for (auto e : m_Voronoi_Hierarchy.m_elist[0])
        {
            if (e->m_break_flag == true)
            {
                Node* startnode;
                Node* nodeold;
                int count = 0;
                int year_step = 2;
                int year_old = 0;
                for (auto kide : e->m_children)
                {
                    cv::Point2d p0, p1;
                    {
                        if (kide->m_verts[0] == NULL)
                        {
                            p0 = cv::Point2d(kide->m_infinit_vert.x, kide->m_infinit_vert.y);
                            p1 = cv::Point2d(kide->m_verts[1]->m_pos.x, kide->m_verts[1]->m_pos.y);
                        }
                        else if (kide->m_verts[1] == NULL)
                        {
                            p0 = cv::Point2d(kide->m_verts[0]->m_pos.x, kide->m_verts[0]->m_pos.y);
                            p1 = cv::Point2d(kide->m_infinit_vert.x, kide->m_infinit_vert.y);
                        }
                        else
                        {
                            p0 = cv::Point2d(kide->m_verts[0]->m_pos.x, kide->m_verts[0]->m_pos.y);
                            p1 = cv::Point2d(kide->m_verts[1]->m_pos.x, kide->m_verts[1]->m_pos.y);
                        }
                    }
                    if (e->m_children.size() == 1)
                    {
                        startnode = new Node(0, p0, 10000000);
                        Node* child = new Node(1, p1, 10000000);
                        child->SetRadius(sqrt(std::max(100, 100)) * 0.07);
                        child->SetParent(startnode);
                        startnode->AppendChild(child);
                    }
                    else if (e->m_children.size() > 1)
                    {
                        Node* nodenow;
                        if (count == 0) //first seed without parent
                        {
                            int random_year = Random::Float() * 10.0;
                            if (kide->m_verts[0] == e->m_children[1]->m_verts[0] || kide->m_verts[0] == e->m_children[1]->m_verts[1])
                                nodenow = new Node(random_year, p1, 10000000);
                            else
                                nodenow = new Node(random_year, p0, 10000000);
                            startnode = nodenow;
                            nodeold = nodenow;
                            year_old = random_year;
                        }
                        else if (count > 0 && count > e->m_children.size() - 1) //behind nodes, set nodes and it's parent
                        {
                            if (kide->m_verts[0] == e->m_children[count - 1]->m_verts[0] || kide->m_verts[0] == e->m_children[count - 1]->m_verts[1])
                                nodenow = new Node(year_old+ year_step, p0, 10000000);
                            else
                                nodenow = new Node(year_old + year_step, p1, 10000000);
                            nodenow->SetRadius(sqrt(std::max(100, 100)) * 0.07);
                            nodenow->SetParent(nodeold);
                            nodeold->AppendChild(nodenow);
                            nodeold = nodenow;
                            year_old = year_old + year_step;
                        }
                        else //last nodes, set nodes and it's parent
                        {
                            Node* finalnode;
                            if (kide->m_verts[0] == e->m_children[count - 1]->m_verts[0] || kide->m_verts[0] == e->m_children[count - 1]->m_verts[1])
                            {
                                nodenow = new Node(year_old + year_step, p0, 10000000);
                                finalnode = new Node(year_old + year_step, p1, 10000000);
                            }
                            else
                            {
                                nodenow = new Node(year_old + year_step, p1, 10000000);
                                finalnode = new Node(year_old + year_step, p1, 10000000);
                            }
                            finalnode->SetRadius(sqrt(std::max(100, 100)) * 0.07);
                            nodenow->SetRadius(sqrt(std::max(100, 100)) * 0.07);
                            nodenow->SetParent(nodeold);
                            nodeold->AppendChild(nodenow);
                            nodeold = nodenow;
                            year_old = year_old + year_step;

                            //append end node of the edge link chain
                            finalnode->SetParent(nodeold);
                            nodeold->AppendChild(finalnode);

                        }
                    }
                    count++;
                }
                m_crackseeds.push_back(startnode);
            }
        }
        m_scndcrackseedindex = m_crackseeds.size();

        //add all the secondary level nodes
        //for (auto e : m_Voronoi_Hierarchy.m_elist[1])
        //{
        //    if (e->m_break_flag == true)
        //    {
        //        cv::Point2d p0, p1;
        //        {
        //            if (e->m_verts[0] == NULL)
        //            {
        //                p0 = cv::Point2d(e->m_infinit_vert.x, e->m_infinit_vert.y);
        //                p1 = cv::Point2d(e->m_verts[1]->m_pos.x, e->m_verts[1]->m_pos.y);
        //            }
        //            else if (e->m_verts[1] == NULL)
        //            {
        //                p0 = cv::Point2d(e->m_verts[0]->m_pos.x, e->m_verts[0]->m_pos.y);
        //                p1 = cv::Point2d(e->m_infinit_vert.x, e->m_infinit_vert.y);
        //            }
        //            else
        //            {
        //                p0 = cv::Point2d(e->m_verts[0]->m_pos.x, e->m_verts[0]->m_pos.y);
        //                p1 = cv::Point2d(e->m_verts[1]->m_pos.x, e->m_verts[1]->m_pos.y);
        //            }
        //        }
        //        Node* startnode = new Node(30, p0, 10000000);
        //
        //        Node* child = new Node(31, p1, 10000000);
        //        child->SetRadius(sqrt(std::max(100, 100)) * 0.07);
        //        child->SetParent(startnode);
        //        startnode->AppendChild(child);
        //
        //        m_crackseeds.push_back(startnode);
        //
        //    }
        //}
        ArrangeSecondaryCrack();
    }
    DrawCrack();
}

double MyVoronoi::Uniformlength(double l)
{
    return l / 1000.0 * 5.0;
}

bool cmp_by_value(const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
    return lhs.second > rhs.second;
}

void MyVoronoi::ArrangeSecondaryCrack()
{
    for (auto c : m_Voronoi_Hierarchy.m_clist[0])
    {
        bool flagcontinue = false; //incase some of broken edge do not have kids, should jump corresponding cell
        for (auto e : c->m_edges)
        {
            if (e->m_break_flag == true && e->m_children.size() != 0)
            {
                flagcontinue = true;
                break;
            }
        }
    
        std::vector<int> c_childedges; //do not include the edge with infinit vertex, all the broken edges //save index of the sub edge

        if (flagcontinue)
        {
            Eigen::MatrixXd g_Nodes;  //edge_vertex_pos 
            Eigen::MatrixXi g_Edges;  //all the edges in one cell

            std::vector<std::pair<double, double>> nodes; //save global position of vertex
            std::vector<std::pair<int, int>> edges; //save vertex local index of edge// first vertex local index, second vertex local index
            std::map<int, int> dic_vert_global_local; //first:global index of vertex  second: local index of vertex
            std::map<int, int> dic_vert_loacal_global;
            std::unordered_map<int, int> dic_traverse_kidedge; //sublevel edge index for not duplicate use, first: global index, second: local index
            std::unordered_map<int, int> dic_edge_local_global;

            //figure out matrix size
            int count = 0;
            for (auto kc : c->m_children)
            {
                for (auto ekid : kc->m_edges)
                {
                    if (dic_traverse_kidedge.find(ekid->m_index) == dic_traverse_kidedge.end())
                    {
                        //here we only record the edge whose is finit
                        if (!ekid->IsInfinit())
                        {
                            dic_traverse_kidedge[ekid->m_index] = count;
                            dic_edge_local_global[count] = ekid->m_index;
                            if (ekid->m_break_flag == true && ekid->m_parent == NULL) //break subedges, the subedge is not on the boundary
                                c_childedges.push_back(ekid->m_index);

                            //for vnodes;
                            vec2 vert0 = ekid->m_verts[0]->m_pos;
                            vec2 vert1 = ekid->m_verts[1]->m_pos;
                            int vert0_index = ekid->m_verts[0]->m_index;
                            int vert1_index = ekid->m_verts[1]->m_index;
                            std::pair<double, double> vert0n = { Uniformlength(vert0.x), Uniformlength(vert0.y) };
                            std::pair<double, double> vert1n = { Uniformlength(vert1.x), Uniformlength(vert1.y) };

                            if (dic_vert_global_local.find(vert0_index) == dic_vert_global_local.end())
                            {
                                dic_vert_global_local[vert0_index] = nodes.size();
                                dic_vert_loacal_global[nodes.size()] = vert0_index;
                                nodes.push_back(vert0n);
                            }
                            if (dic_vert_global_local.find(vert1_index) == dic_vert_global_local.end())
                            {
                                dic_vert_global_local[vert1_index] = nodes.size();
                                dic_vert_loacal_global[nodes.size()] = vert1_index;
                                nodes.push_back(vert1n);
                            }

                            edges.push_back({ dic_vert_global_local[vert0_index] , dic_vert_global_local[vert1_index] });
                            count++;
                        }
                    }
                }
            }
            g_Nodes.resize(nodes.size(), 2);
            g_Edges.resize(edges.size(), 2);
            for (int i = 0; i < nodes.size(); i++)
            {
                g_Nodes(i, 0) = nodes[i].first;
                g_Nodes(i, 1) = nodes[i].second;
            }
            for (int i = 0; i < edges.size(); i++)
            {
                g_Edges(i, 0) = edges[i].first;
                g_Edges(i, 1) = edges[i].second;
            }


            HSS::PathFinder pathfinder(g_Nodes, g_Edges);
            
            for (int i = 0; i < edges.size(); i++)
            {
                if (m_Voronoi_Hierarchy.m_elist[1][dic_edge_local_global[i]]->m_parent != NULL)
                {
                    //avoid the secondary path pass the big boundary (except for the break nodes)
                    int BreakNodeonBigEdge_index = m_Voronoi_Hierarchy.m_elist[1][dic_edge_local_global[i]]->m_parent->m_secondarybreakpoint[c->m_index];
                    int small_edge_vertex_index1 = m_Voronoi_Hierarchy.m_vlist[1][dic_vert_loacal_global[edges[i].first]]->m_index;
                    int small_edge_vertex_index2 = m_Voronoi_Hierarchy.m_vlist[1][dic_vert_loacal_global[edges[i].second]]->m_index;
                    if (small_edge_vertex_index1 != BreakNodeonBigEdge_index)
                        pathfinder.EnableNode(edges[i].first, false);
                    if (small_edge_vertex_index2 != BreakNodeonBigEdge_index)
                        pathfinder.EnableNode(edges[i].second, false);
                }
                else
                {
                    //for each break sub edges
                    if (m_Voronoi_Hierarchy.m_elist[1][dic_edge_local_global[i]]->m_break_flag == true)
                    {
                        std::vector<int> src;
                        src.push_back(edges[i].first);
                        src.push_back(edges[i].second);
                        std::vector<double> dists;
                        std::vector<int> spanTree;
                        pathfinder.Dijkstras(src, dists, spanTree);

                        //save the path from the subedge to big edge node in the subedge
                        for (auto e : c->m_edges)
                        {
                            if (e->m_break_flag == true && e->m_children.size() != 0)
                            {
                                int breakpoint_index = e->m_secondarybreakpoint[c->m_index];
                                int targetIdx = dic_vert_global_local[breakpoint_index];
                                int nid = targetIdx;
                                int noldid = nid;
                                while (nid != -1)
                                {
                                    m_Voronoi_Hierarchy.m_elist[1][dic_edge_local_global[i]]->m_toboudarypath[e->m_index].push_back(dic_vert_loacal_global[nid]);
                                    vec2 pold = m_Voronoi_Hierarchy.m_vlist[1][dic_vert_loacal_global[noldid]]->m_pos;
                                    vec2 pnew = m_Voronoi_Hierarchy.m_vlist[1][dic_vert_loacal_global[nid]]->m_pos;
                                    double distance = glm::length(pold - pnew);
                                    m_Voronoi_Hierarchy.m_elist[1][dic_edge_local_global[i]]->m_toboudaryenergy[e->m_index] += distance;
                                    noldid = nid;
                                    nid = spanTree[nid];
                                }
                            }
                        }
                    }
                }
            }
        }
        //add seeds for the secondary crack
        for (auto e : c->m_edges)
        {
            double potential_energy = e->m_potential_energy*0.05; //pitch:0.2 //light mud: 0.3 //cement,wood: 0.05
            if (e->m_break_flag == true && e->m_children.size() != 0)
            {
                //sorted energy of all those small break edges
                int breakpoint_index = e->m_secondarybreakpoint[c->m_index];
                std::vector<std::pair<int,double>> sorted_energy; //saved the possible path energy from high to low // index is the edge index and its secondary crack energy
                for (int i = 0; i < c_childedges.size(); i++)
                {
                    sorted_energy.push_back({ c_childedges[i], m_Voronoi_Hierarchy.m_elist[1][c_childedges[i]]->m_toboudaryenergy[e->m_index]});
                }
                sort(sorted_energy.begin(), sorted_energy.end(), cmp_by_value);

                //saved global index of small break edges for secondary crack
                std::vector<int> crack_edges; 
                double gamma_s = 0.04936;//https://www.scielo.br/j/rmat/a/nmYq47WY79jbjTMJVQDMwnG/?lang=en#
                for (auto k : sorted_energy)
                {
                    crack_edges.push_back(k.first);
                    double Ws = 2.0 * gamma_s * k.second;
                    potential_energy -= Ws;
                    if (potential_energy <= 0)
                        break;
                }

                //create seeds
                for (auto picked_edge : crack_edges)
                {
                    Node* Nownode;
                    Node* Parentnode;
                    int count = 0;
                    for (auto ppoint : m_Voronoi_Hierarchy.m_elist[1][picked_edge]->m_toboudarypath[e->m_index])
                    {
                        vec2 ppos = m_Voronoi_Hierarchy.m_vlist[1][ppoint]->m_pos;
                        
                        if (count == 0) 
                        {
                            Parentnode = new Node(30, cv::Point2d(ppos.x, ppos.y), 10000000);
                            m_crackseeds.push_back(Parentnode);
                        }
                        else
                        {
                            Nownode = new Node(Parentnode->Return_Year()+1, cv::Point2d(ppos.x, ppos.y), 10000000);
                            Nownode->SetParent(Parentnode);
                            Parentnode->AppendChild(Nownode);
                            Parentnode = Nownode;
                        }
                        count++;
                    }
                }
            }
        }
    }
}

void MyVoronoi::IncreaseBranch(int erostime)
{
    if (m_FDE_Mesh != NULL)
    {
        //=====================Render the crack======================//
        //if (erostime % 1 == 0 && erostime > m_old_time) // lauch interval: 0 lauch situation: time is increasing
        //{
        //    CreateCrack();
        //    m_old_time = erostime;
        //}
        //for (int crackindex = 0; crackindex < m_crackseeds.size(); crackindex++)
        //{
        //    RenderCrack(erostime, m_crackseeds[crackindex]);
        //    m_garbage_vets[m_crackseeds[crackindex]->Return_Vertex()->m_index] = 0;
        //    DrawCrack();
        //}
        m_old_time = erostime;
        TestDrawPriandSecEdge();

        cv::Mat Black_White = cv::Mat::zeros(m_row, m_column, CV_8UC1); 
        cv::cvtColor(m_vmap, Black_White, cv::COLOR_BGR2GRAY);
        std::ostringstream os;
        os << "./" << m_row << "x" << m_column << "_black_white.png";
        cv::imwrite(os.str(), Black_White);  // This image is for render the crack light normal

        m_vmap.copyTo(m_rendermap);
        cv::Mat copy_red = cv::Mat::zeros(m_row, m_column, CV_8UC3);
        cv::Mat copy_blue = cv::Mat::zeros(m_row, m_column, CV_8UC3);

        for (int i = 0; i < m_row; i++)
        {
            for (int j = 0; j < m_column; j++)
            {
                cv::Vec3b tempb = copy_blue.at<cv::Vec3b>(cv::Point(i, j));
                cv::Vec3b tempr = copy_red.at<cv::Vec3b>(cv::Point(i, j));
                tempb[0] = 255 - Black_White.at<uchar>(j, i);              
                tempr[2] = 255 - Black_White.at<uchar>(j, i);
                copy_blue.at<cv::Vec3b>(cv::Point(i, j)) = tempb;    
                copy_red.at<cv::Vec3b>(cv::Point(i, j)) = tempr;     
            }
        }
        cv::imwrite("red.jpg", copy_red);
        cv::imwrite("blue.jpg", copy_blue);

        cv::Mat copy_blur = cv::Mat::zeros(m_row, m_column, CV_8UC3);
        copy_red.copyTo(copy_blur);
        cv::blur(copy_blur, copy_blur, cv::Size(10, 10));
        //GaussianBlur(copy_blur, copy_blur, cv::Size(5, 5), 0);
        cv::imwrite("red_blur.jpg", copy_blur);
        
        cv::Mat diff_img = cv::Mat::zeros(m_row, m_column, CV_8UC3);
        cv::absdiff(copy_red, copy_blur, diff_img);
        cv::imwrite("red_blur_diff.jpg", diff_img);

        cv::add(diff_img, copy_blue, m_rendermap);

        cv::imwrite("./CrackMaps/Damage_Holes.png", m_rendermap);  // This image is for render the errosion surrounding the crack

        std::cout << "Saved image as " << os.str() << "\n";
    }
    else qWarning() << "Please Set the temperature field first (Step 2) [from damage].";
}


//bool MyVoronoi::AddChildren(Node* pnode)
//{
//    //traverse all the edges of a vertex
//    std::vector<HVD::vert*> endpoint_garage;  //1-distance neighbor vertex of now vertex
//    std::vector<HVD::edge*> corres_edge_garage;  //corresponding edge of the endpoint
//
//    for (auto edge : pnode->Return_Vertex()->m_edges)
//    {
//        if (edge->m_infinit_vert == vec2(-1,-1))  //only consider the voronoi edges 
//        {
//            if (edge->m_verts[0]->m_index == pnode->Return_Vertex()->m_index && m_garbage_vets.find(edge->m_verts[1]->m_index) == m_garbage_vets.end())
//            {
//                endpoint_garage.push_back(edge-> m_verts[1]);
//                corres_edge_garage.push_back(edge);
//            }
//            else if (edge->m_verts[1]->m_index == pnode->Return_Vertex()->m_index && m_garbage_vets.find(edge->m_verts[0]->m_index) == m_garbage_vets.end())
//            {
//                endpoint_garage.push_back(edge->m_verts[0]);
//                corres_edge_garage.push_back(edge);
//            }
//        }
//    }
//
//    // randomly add one or two children:
//    int random_num_child;
//    if (pnode->Return_Year() < 4 ||pnode->Return_Year()%4 != 0)
//        random_num_child = 1.0;
//    else random_num_child = Random::Float() * 2.2;
//    //random_num_child = 1;  //1;
//
//    double old_potential = pnode->ReturnPotential();  // get the antient's growth potential;
//    if (endpoint_garage.size() >= random_num_child)
//    {
//        for (int i = 0; i < random_num_child; i++)
//        {
//            Node* child;
//
//            //if (node->Return_Year() == 0)
//            //{
//            //    int random_direction = Random::Float() * endpoint_garage.size();
//            //    child = new Node(node->Return_Year() + 1, endpoint_garage[random_direction], potential, random_direction);  //decide a random direction
//            //}
//            //else
//            //{ 
//            //   int direction = std::min(node->GetDirection(), i);
//            //    child = new Node(node->Return_Year() + 1, endpoint_garage[direction], potential, direction);  //follow the first direction
//            //}
//            double x = pnode->Return_Vertex()->m_pos.x;
//            double y = pnode->Return_Vertex()->m_pos.y;
//            double canvasX = m_FDE_Mesh->ReturnWidth();
//            double canvasY = m_FDE_Mesh->ReturnHeight();
//            double inputX = (x / (double)m_column * canvasX) - 0.5 * canvasX;
//            double inputY = (y / (double)m_row * canvasY) - 0.5 * canvasY;
//            //auto gradient = m_Temp_Grid->CalculateGradient(inputX, inputY);
//            double angle = 180.0; //+ (180.0 + gradient.first);
//            double direction_angle_1 = angle - floor(angle / 360.0) * 360.0;
//            double direction_angle_2 = (angle + 180.0) - floor((angle + 180.0) / 360.0) * 360.0;
//            int direction_index = -1;
//            double now_distance = 360;
//            //m_row = rows;
//
//            for (int i = 0; i < endpoint_garage.size(); i++)  // go to the min delta angle direction
//            {
//                if (endpoint_garage[i]->m_pos.x >= 0 && endpoint_garage[i]->m_pos.y >= 0 && endpoint_garage[i]->m_pos.x <= m_column && endpoint_garage[i]->m_pos.y <= m_row)
//                {
//                    vec2 end = vec2(endpoint_garage[i]->m_pos.x, endpoint_garage[i]->m_pos.y);
//                    vec2 start = vec2(pnode->Return_Vertex()->m_pos.x, pnode->Return_Vertex()->m_pos.y);
//                    vec2 vector = end - start;
//                    double ang = atan2(vector.y, vector.x) * 180.0 / 3.1415916;
//                    double a = (ang + 180.0);
//                    if (abs(a - direction_angle_1) < now_distance || abs(a - direction_angle_2) < now_distance)
//                    {
//                        now_distance = std::min(abs(a - direction_angle_1), abs(a - direction_angle_2));
//                        direction_index = i;
//                    }
//                }
//            }
//            if (direction_index != -1)
//            {
//                //double K_time_decay = 1;
//                //double sigma = 0.05;
//                //double L_area, R_area;
//                //const voronoi_diagram<double>::cell_type* cell_1 = corres_edge_garage[direction_index]->cell();
//                //const voronoi_diagram<double>::cell_type* cell_2 = corres_edge_garage[direction_index]->twin()->cell();
//                //R_area = CalculateArea(cell_1);
//                //L_area = CalculateArea(cell_2);
//                //double l = sqrt(pow(endpoint_garage[direction_index]->x() - node->Return_Vertex()->x(), 2)
//                //    + pow(endpoint_garage[direction_index]->y() - node->Return_Vertex()->y(), 2));
//                //double Tao_stress = sigma*(L_area + R_area);
//                //double ita = K_time_decay / Tao_stress; /// + ita_pos
//                //
//                //double child_potential = old_potential - ita * l;
//
//
////==========================================NEED SOME CHANGE=======================================================//
//                //vec2 end = vec2(endpoint_garage[direction_index]->x(), endpoint_garage[direction_index]->y());
//                //vec2 start = vec2(node->Return_Vertex()->x(), node->Return_Vertex()->y());
//                //vec2 vector = end - start;
//                //double length = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2));
//                //if (length < node->ReturnRadius()) return false;
//                //else {
//                double L_area, R_area;
//                HVD::cell* cell_1 = corres_edge_garage[direction_index]->m_cells[0];
//                HVD::cell* cell_2 = corres_edge_garage[direction_index]->m_cells[1];
//                R_area = 100;//CalculateArea(cell_1);
//                L_area = 100;//CalculateArea(cell_2);
//                double child_potential = old_potential - 100 / std::max(R_area, L_area); // large area easy for crack to stay alive
//                child = new Node(pnode->Return_Year() + 1, endpoint_garage[direction_index], child_potential);
//                
//                child->SetRadius(sqrt(std::max(R_area, L_area)) * 0.07);
//
//                ////========== checking whether the new child's curve is toO sharp ========//
//                //vec2 end = vec2(endpoint_garage[direction_index]->x(), endpoint_garage[direction_index]->y());
//                //bool sharp_flag = false;
//                //Node* it = node->ReturnParent();
//                //while (it != NULL)
//                //{
//                //    vec2 start = vec2(it->Return_Vertex()->x(), it->Return_Vertex()->y());
//                //    double length = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2));
//                //    if (length < child->ReturnRadius())
//                //    {
//                //        sharp_flag = true;
//                //        break;
//                //    }
//                //    it = it->ReturnParent();
//                //}
//                //if (sharp_flag)
//                //{ 
//                //    child->~Node();
//                //    return false;
//                //}
//                //else
//                //{
//                child->SetParent(pnode);
//                pnode->AppendChild(child);
//                //}
//            //}
//
//            //=========================Add cousin:===========================//
//
//
//
//            }
//            else
//            {
//                qWarning() << "CANNOT find edges that is both finit and visible for the future direction.";
//                return false;
//            }
//        }
//        return true;
//    }
//    else return false;
//}
bool MyVoronoi::AddChildren(Node* pnode)
{
    //traverse all the edges of a vertex
    std::vector<HVD::vert*> endpoint_garage;  //1-distance neighbor vertex of now vertex
    std::vector<HVD::edge*> corres_edge_garage;  //corresponding edge of the endpoint
    std::vector<HVD::vert*> endpoint_garage_have_parent;  //1-distance neighbor vertex of now vertex
    std::vector < HVD::edge* > corres_edge_garage_have_parent;
    
    for (auto edge : pnode->Return_Vertex()->m_edges)
    {
        if (edge->m_infinit_vert == vec2(-1, -1))  //only consider the voronoi edges 
        {
            if (edge->m_verts[0]->m_index == pnode->Return_Vertex()->m_index && m_garbage_vets.find(edge->m_verts[1]->m_index) == m_garbage_vets.end())
            {
                
                if (edge->m_parent != NULL)
                {
                    endpoint_garage_have_parent.push_back(edge->m_verts[1]);
                    corres_edge_garage_have_parent.push_back(edge);
                }
                else
                {
                    endpoint_garage.push_back(edge->m_verts[1]);
                    corres_edge_garage.push_back(edge);
                }
            }
            else if (edge->m_verts[1]->m_index == pnode->Return_Vertex()->m_index && m_garbage_vets.find(edge->m_verts[0]->m_index) == m_garbage_vets.end())
            {
                
                if (edge->m_parent != NULL)
                {
                    endpoint_garage_have_parent.push_back(edge->m_verts[0]);
                    corres_edge_garage_have_parent.push_back(edge);
                }
                else
                {
                    endpoint_garage.push_back(edge->m_verts[0]);
                    corres_edge_garage.push_back(edge);
                }
            }
        }
    }

    // randomly add one or two children:
    int random_num_child;
    if (pnode->Return_Year() > 56 && pnode->Return_Year()%4 ==0)
        random_num_child = 1.0;
    else random_num_child = 0.0;//Random::Float() * 2.2;
    //random_num_child = 1;  //1;

    bool success_flag = false;

    double old_potential = pnode->ReturnPotential();  // get the antient's growth potential;
    if (endpoint_garage.size() >= random_num_child)
    {
        for (int i = 0; i < random_num_child; i++)
        {
            Node* child;
                double child_potential = old_potential - 1; // large area easy for crack to stay alive
                child = new Node(pnode->Return_Year() + 1, endpoint_garage[i], child_potential);
    
                child->SetRadius(sqrt(std::max(100, 100)) * 0.07);
    
                child->SetParent(pnode);
                pnode->AppendChild(child);
        }
        success_flag = true;
    }

    for (int i = 0; i < endpoint_garage_have_parent.size(); i++)
    {
        Node* child;

        double child_potential = old_potential - 0.1; // large area easy for crack to stay alive
        child = new Node(pnode->Return_Year() + 1, endpoint_garage_have_parent[i], child_potential);

        child->SetRadius(sqrt(std::max(100, 100)) * 0.07);

        child->SetParent(pnode);
        pnode->AppendChild(child);
        success_flag = true;
    }
    return success_flag;

}


bool MyVoronoi::CheckMergeDamageHole(Node* node)
{
    std::vector<cv::Point> elementPoints;

    for (auto damagepoint : m_damageseeds)
    {
        const voronoi_diagram<double>::cell_type& cell = *damagepoint;
        const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();

        if (edge->is_finite())
        {
            if (node->Return_Vertex()->m_pos.x == edge->vertex0()->x() && node->Return_Vertex()->m_pos.y == edge->vertex0()->y())
            {
                if (edge->twin()->next()->is_finite() && edge->prev()->is_finite())
                {
                    cv::Point BEGIN = cv::Point(edge->vertex0()->x(), edge->vertex0()->y());
                    cv::Point END = cv::Point(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                    cv::Point Want = BEGIN + (float) 0.2 * (END - BEGIN);
                    elementPoints.push_back(Want);
                    elementPoints.push_back(cv::Point(edge->vertex1()->x(), edge->vertex1()->y()));
                    elementPoints.push_back(cv::Point(edge->prev()->vertex0()->x(), edge->prev()->vertex0()->y()));
                    cv::fillPoly(m_vmap, elementPoints, cv::Scalar(0, 0, 0), 8);
                }
                return true;
            }
        }
        do {
            edge = edge->next();
            {
                if (edge->is_finite())
                {
                    if (node->Return_Vertex()->m_pos.x == edge->vertex0()->x() && node->Return_Vertex()->m_pos.y == edge->vertex0()->y())
                    {
                        if (edge->twin()->next()->is_finite() && edge->prev()->is_finite())
                        {
                            cv::Point BEGIN = cv::Point(edge->vertex0()->x(), edge->vertex0()->y());
                            cv::Point END = cv::Point(edge->twin()->next()->vertex1()->x(), edge->twin()->next()->vertex1()->y());
                            cv::Point Want = BEGIN + (float)0.2 * (END - BEGIN);
                            elementPoints.push_back(Want);
                            elementPoints.push_back(cv::Point(edge->vertex1()->x(), edge->vertex1()->y()));
                            elementPoints.push_back(cv::Point(edge->prev()->vertex0()->x(), edge->prev()->vertex0()->y()));
                            cv::fillPoly(m_vmap, elementPoints, cv::Scalar(0, 0, 0), 8);
                        }
                        return true;
                    }
                }
            }
        } while (edge != cell.incident_edge());  
    }
    return false;
}



void MyVoronoi::RenderCrack(int erostime, Node* node)
{
    //crack parts
    if (node->Return_Vertex() != NULL)
    {
        if (erostime == node->Return_Year() + 1)  // only need to render a exist part of the node
        {
            for (auto child : node->Return_Children())
            {
                if (node->ReturnParent() != NULL) //after initial
                {
                    vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
                        node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
                    vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
                        child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
                    double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                    if (arc >= (3.1415926 / 2.5)) continue;
                    else
                    {
                        DrawLine(node, child);
                    }
                }
                else // initial
                {
                    DrawLine(node, child);
                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                }
            }
        }
        else if (erostime > node->Return_Year() + 1 && node->ReturnPotential() > m_destroy_potential)  //node->Return_Year() < node->ReturnPotential() //20 is the maximum growth height of each tree
        {
            if (node->Return_Children().size() == 0)
            {
                if (AddChildren(node) == true)
                {
                    for (auto child : node->Return_Children())
                    {
                        if (node->ReturnParent() != NULL) //after initial
                        {
                            vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
                                node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
                            vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
                                child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
                            double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
                            m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                            if (!CheckMergeDamageHole(child)) 
                                RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
                            if (arc >= (3.1415926 / 2.5)) continue;
                            else
                            {
                                DrawLine(node, child);
                            }
                            
                        }
                        else // initial
                        {
                            DrawLine(node, child);
                            m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                            if (!CheckMergeDamageHole(child)) 
                                RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
                        }
                        
                    }
                }
                else return;
            }
            else
            {
                for (auto child : node->Return_Children())
                {
                    if (node->ReturnParent() != NULL) //after initial
                    {
                        vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
                            node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
                        vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
                            child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
                        double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                        if (!CheckMergeDamageHole(child)) 
                            RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
                        if (arc >= (3.1415926 / 2.5)) continue;
                        else
                        {
                            DrawLine(node, child);
                        }
                        
                    }
                    else // initial
                    {
                        DrawLine(node, child);
                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                        if (!CheckMergeDamageHole(child)) 
                            RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
                    }
                    
                }
            }
        }
    }
}

//void MyVoronoi::RenderCrack(int erostime, Node* node)
//{
//    //crack parts
//    if (node->Return_Vertex() != NULL)
//    {
//        if (erostime == node->Return_Year() + 1)  // only need to render a exist part of the node
//        {
//            for (auto child : node->Return_Children())
//            {
//                if (node->ReturnParent() != NULL) //after initial
//                {
//                    vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
//                        node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
//                    vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
//                        child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
//                    double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
//                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                    if (arc >= (3.1415926 / 2.5)) continue;
//                    else
//                    {
//                        DrawLine(node, child);
//                    }
//                }
//                else // initial
//                {
//                    DrawLine(node, child);
//                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                }
//            }
//        }
//        else if (erostime > node->Return_Year() + 1 && node->ReturnPotential() > m_destroy_potential)  //node->Return_Year() < node->ReturnPotential() //20 is the maximum growth height of each tree
//        {
//            if (node->Return_Children().size() == 0)
//            {
//                if (AddChildren(node) == true)
//                {
//                    for (auto child : node->Return_Children())
//                    {
//                        if (node->ReturnParent() != NULL) //after initial
//                        {
//                            vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
//                                node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
//                            vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
//                                child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
//                            double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
//                            m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                            if (!CheckMergeDamageHole(child))
//                                RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
//                            if (arc >= (3.1415926 / 2.5)) continue;
//                            else
//                            {
//                                DrawLine(node, child);
//                            }
//
//                        }
//                        else // initial
//                        {
//                            DrawLine(node, child);
//                            m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                            if (!CheckMergeDamageHole(child))
//                                RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
//                        }
//
//                    }
//                }
//                else return;
//            }
//            else
//            {
//                for (auto child : node->Return_Children())
//                {
//                    if (node->ReturnParent() != NULL) //after initial
//                    {
//                        vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
//                            node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
//                        vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
//                            child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
//                        double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
//                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                        if (!CheckMergeDamageHole(child))
//                            RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
//                        if (arc >= (3.1415926 / 2.5)) continue;
//                        else
//                        {
//                            DrawLine(node, child);
//                        }
//
//                    }
//                    else // initial
//                    {
//                        DrawLine(node, child);
//                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
//                        if (!CheckMergeDamageHole(child))
//                            RenderCrack(erostime, child); // check whether the crack has been merged into the damage hole
//                    }
//
//                }
//            }
//        }
//    }
//}

void MyVoronoi::RenderDamageCrack(Node* node)
{
    if ( node->ReturnPotential() > m_destroy_potential)  
    {
        if (node->Return_Children().size() == 0)
        {
            if (AddChildren(node) == true)
            {
                for (auto child : node->Return_Children())
                {
                    if (node->ReturnParent() != NULL) //after initial
                    {
                        vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
                            node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
                        vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
                            child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
                        double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                        RenderDamageCrack(child);
                        if (arc >= (3.1415926 / 2.5)) continue;
                        else DrawLine(node, child);
                    }
                    else // initial
                    {
                        DrawLine(node, child);
                        m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                        RenderDamageCrack(child);
                    }
                }
            }
            else return;
        }
        else
        {
            for (auto child : node->Return_Children())
            {
                if (node->ReturnParent() != NULL) //after initial
                {
                    vec2 line_old = vec2(node->Return_Vertex()->m_pos.x - node->ReturnParent()->Return_Vertex()->m_pos.x,
                        node->Return_Vertex()->m_pos.y - node->ReturnParent()->Return_Vertex()->m_pos.y);
                    vec2 line_new = vec2(child->Return_Vertex()->m_pos.x - node->Return_Vertex()->m_pos.x,
                        child->Return_Vertex()->m_pos.y - node->Return_Vertex()->m_pos.y);
                    double arc = glm::acos(glm::dot(glm::normalize(line_old), glm::normalize(line_new)));
                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                    RenderDamageCrack(child);
                    if (arc >= (3.1415926 / 2.5)) continue;
                    else DrawLine(node, child);
                }
                else // initial
                {
                    DrawLine(node, child, 1);
                    m_garbage_vets[child->Return_Vertex()->m_index] = child->Return_Year();
                    RenderDamageCrack(child);
                }

            }
        }
    }
    else return; 
}

void GradientCircle(cv::Mat m, cv::Point center, int radius)
{
    cv::circle(m, center, radius - 1, cv::Scalar(0, 0, 0), -1);

    for (double i = 1.0 / radius; i <= 1.0; i += 1.0 / radius)
    {
        int color = 0 + int(pow(i, 3) * 100); //or some another color calculation
        int q = i * radius;
        cv::circle(m, center, q, cv::Scalar(color, color, color), 2);
    }

}

void GradientCircle2(cv::Mat m, cv::Point center, int radius)
{
    cv::circle(m, center, radius - 1, cv::Scalar(0, 0, 0), -1);

    for (double i = 1.0 / radius; i <= 1.0; i += 1.0 / radius)
    {
        int color = 0 + int(pow(i, 0.4) * 200); //or some another color calculation
        int q = i * radius;
        cv::circle(m, center, q, cv::Scalar(color, color, color), 2);
    }

}


void drawCross(cv::Mat img, cv::Point point, int size, cv::Scalar color, int thickness)
{
    //horozontal
    cv::line(img, cv::Point(point.x - size / 2, point.y), cv::Point(point.x + size / 2, point.y), color, thickness, 8, 0);
    //vertical
    cv::line(img, cv::Point(point.x, point.y - size / 2), cv::Point(point.x, point.y + size / 2), color, thickness, 8, 0);
    return;
}

void OutPutExcel(std::string m_ImageName, std::vector<double> value)
{
    std::ofstream outf;
    std::string s1 = m_ImageName;
    std::string s0("./Fit_curve_data/");
    std::string s2(".csv");
    s0.append(s1);
    s0.append(s2);
    outf.open(s0, std::ios::out | std::ios::trunc);
    if (!outf.is_open())
        qDebug() << "Error when do out put";

    outf << "Picture Name" << "," << "Degree" << "," << "Value (Accumulated saturation for each degree of hue)" << "," << "TEMP1 range" << "," << "TEMP2 range" << "," << "TEMP name" << std::endl;


    outf << m_ImageName << "," << "," << "," << "x" << " " << "x" << "," << "x" << " " << "x" << "," << "x" << std::endl;
    for (int i = 0; i < value.size(); i++)
    {
        outf << "," << i << "," << value[i] << std::endl;
    }

    outf.close();
}

void MyVoronoi::DrawRuler(cv::Mat m)
{
    int height = m_row;
    int width = m_column;
    cv::circle(m, Site(19, 19), 3, cv::Scalar(0, 0, 0), -1);
    putText(m, std::to_string(0), Site(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, false);

    for (int y = 19; y < 429; y += 5)
    {
        if ((y + 1) % 40 == 10)
        {
            cv::line(m, Site(19, y), Site(35, y), cv::Scalar(0, 0, 0), 1);
            putText(m, std::to_string(y - 19), Site(40, y + 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, false);
        }
        else if ((y + 1) % 40 == 30)
        {
            if (y <= 190)
                putText(m, std::to_string(y - 19), Site(31, y + 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, false);
            cv::line(m, Site(19, y), Site(26, y), cv::Scalar(0, 0, 0), 1);
        }
    }
    cv::arrowedLine(m, Site(19, 19), Site(19, 439), cv::Scalar(0, 0, 0), 2, 8, 0, 0.015);
    putText(m, "Pixel Size", Site(3, 459), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, false);
}

void MyVoronoi::CalHomogeneous()
{
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<int> x(181);
    std::vector<double> Angle_garage(181, 0);
    double total_numbers = 0;
    double sum = 0;

    for (voronoi_diagram<double>::const_vertex_iterator it = m_vd.vertices().begin(); it != m_vd.vertices().end(); ++it)
    {
        const voronoi_diagram<double>::vertex_type& vertex = *it;
        const voronoi_diagram<double>::edge_type* edge = it->incident_edge();
        vec2 vertex_pos = vec2(vertex.x(), vertex.y());
        vec2 p1;
        vec2 p2;
        do // This is convenient way to iterate edges around Voronoi vertex.
        {
            if (edge->is_primary())
            {
                if (edge->is_finite())
                {
                    p1 = vec2(edge->vertex1()->x(), edge->vertex1()->y());

                    if (edge->rot_next()->is_finite()) p2 = vec2(edge->rot_next()->vertex1()->x(), edge->rot_next()->vertex1()->y());
                    else
                    {
                        Site p1t = m_sites[edge->rot_next()->cell()->source_index()];
                        Site p2t = m_sites[edge->rot_next()->twin()->cell()->source_index()];
                        int end_x = (p1t.y - p2t.y) * 2;
                        int end_y = (p2t.x - p1t.x) * 2;
                        p2 = vertex_pos + vec2(end_x, end_y);
                    }
                }
                else
                {
                    Site p1s = m_sites[edge->cell()->source_index()];
                    Site p2s = m_sites[edge->twin()->cell()->source_index()];
                    int end_xs = (p1s.y - p2s.y) * 2;
                    int end_ys = (p2s.x - p1s.x) * 2;
                    p1 = vertex_pos + vec2(end_xs, end_ys);

                    if (edge->rot_next()->is_finite()) p2 = vec2(edge->rot_next()->vertex1()->x(), edge->rot_next()->vertex1()->y());
                    else
                    {
                        Site p1t = m_sites[edge->rot_next()->cell()->source_index()];
                        Site p2t = m_sites[edge->rot_next()->twin()->cell()->source_index()];
                        int end_x = (p1t.y - p2t.y) * 2;
                        int end_y = (p2t.x - p1t.x) * 2;
                        p2 = vertex_pos + vec2(end_x, end_y);
                    }
                }
                //int angle = getAngelOfTwoVector(p1, p2, vertex_pos);
                int angle;
                vec2 v1 = normalize(vec2(p1.x - vertex_pos.x, p1.y - vertex_pos.y));
                vec2 v2 = normalize(vec2(p2.x - vertex_pos.x, p2.y - vertex_pos.y));
                double theta;
                double dotp = dot(v1, v2);
                theta = acos(dotp);

                angle = round(theta * 180.0 / CV_PI);
                Angle_garage[angle] += 1;
                total_numbers += 1;
            }
            edge = edge->rot_next();
        } while (edge != it->incident_edge());
    }

    std::vector<double> frequency(181, 0);
    frequency = Angle_garage;
    for (size_t i = 0; i < x.size(); i++) {
        x[i] = i;
        sum += (double)i * Angle_garage[i];
        frequency[i] /= total_numbers;
    }
    auto axes = CvPlot::makePlotAxes();
    axes.create<CvPlot::Series>(x, frequency, "-r");
    axes.title("The frequency distribution histogram of vertex angles");

    double mean = sum / total_numbers;
    double sq_sum = 0;
    for (size_t j = 0; j < x.size(); j++) {
        sq_sum += (pow(((double)j - mean), 2) * Angle_garage[j]);
    }
    double stdev = std::sqrt(sq_sum / total_numbers);

    axes.xLabel("Alpha (Degree)");
    axes.yLabel("Relative Frequency (%)");
    //axes.create<MyRect>();
    cv::Mat mat = axes.render(1000, 1500);
    std::string var1 = "Mean = " + std::to_string(mean);
    std::string var2 = "Standard Deviation = " + std::to_string(stdev);
    std::string var3 = "Total Vertices Number = " + std::to_string(total_numbers);
    cv::putText(mat, var1, cv::Point(250, 200), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat, var2, cv::Point(250, 250), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat, var3, cv::Point(250, 300), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);

    cv::imwrite("./Statistical_data/Statistical_data_angle.png", mat);
    OutPutExcel("Statistical_data_angle", frequency);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<int> l(400);
    std::vector<double> length_garage(400, 0);
    std::unordered_map<const boost::polygon::voronoi_edge<double>*, int> eeedge_garage;
    int maxlength = 0;
    int minlength = 1000;
    double length_total_numbers = 0;
    double length_sum = 0;

    for (voronoi_diagram<double>::const_edge_iterator it = m_vd.edges().begin(); it != m_vd.edges().end(); ++it)
    {
        const voronoi_diagram<double>::edge_type& edge = *it;
        if (edge.is_finite() && edge.is_primary() && edge.twin()->is_finite() && edge.twin()->is_primary())
        {
            if (eeedge_garage[&edge] == 0 && eeedge_garage[edge.twin()] == 0)
            {
                vec2 vertex0 = vec2(edge.vertex0()->x(), edge.vertex0()->y());
                vec2 vertex1 = vec2(edge.vertex1()->x(), edge.vertex1()->y());
                int nowlength = round(glm::length(vertex0 - vertex1));
                if (nowlength < 400)
                {
                    length_garage[nowlength] += 1;
                    length_total_numbers += 1;
                    if (length_garage[nowlength] > 5)
                        maxlength = std::max(maxlength, nowlength);
                    //if
                    minlength = std::min(minlength, nowlength);
                }
                eeedge_garage[&edge] += 1;
                eeedge_garage[edge.twin()] += 1;
            }
        }
    }
    std::vector<double> length_frequency(400, 0);
    length_frequency = length_garage;
    for (size_t i = 0; i < l.size(); i++) {
        l[i] = i;
        length_sum += (double)i * length_garage[i];
        length_frequency[i] /= length_total_numbers;
    }
    auto axes2 = CvPlot::makePlotAxes();
    axes2.create<CvPlot::Series>(l, length_frequency, "-r");
    axes2.title("The frequency distribution histogram of edge lengths");

    double length_mean = length_sum / length_total_numbers;
    double length_sq_sum = 0;
    for (size_t j = 0; j < l.size(); j++) {
        length_sq_sum += (pow(((double)j - length_mean), 2) * length_garage[j]);
    }
    double length_stdev = std::sqrt(length_sq_sum / length_total_numbers);

    axes2.xLabel("Length (Pixel Length)");
    axes2.yLabel("Relative Frequency (%)");
    //axes.create<MyRect>();
    cv::Mat mat2 = axes2.render(1000, 1500);
    std::string var21 = "Mean = " + std::to_string(length_mean);
    std::string var22 = "Standard Deviation = " + std::to_string(length_stdev);
    std::string var33 = "Maximum Effective Length = " + std::to_string(maxlength);
    std::string var4 = "Total Edges Number = " + std::to_string(length_total_numbers);
    std::string var5 = "Total Sites Number = " + std::to_string(m_sites.size());
    cv::putText(mat2, var21, cv::Point(800, 200), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var22, cv::Point(800, 250), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var33, cv::Point(800, 300), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var4, cv::Point(800, 350), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var5, cv::Point(800, 400), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::imwrite("./Statistical_data/Statistical_data_length.png", mat2);
    OutPutExcel("Statistical_data_length", length_frequency);
}

void MyVoronoi::CalDamHomogeneous()
{
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<int> x(181);
    std::vector<double> Angle_garage(181, 0);
    double total_numbers = 0;
    double sum = 0;

    for (voronoi_diagram<double>::const_vertex_iterator it = m_Voronoi_Damage_Diagram.vertices().begin(); it != m_Voronoi_Damage_Diagram.vertices().end(); ++it)
    {
        const voronoi_diagram<double>::vertex_type& vertex = *it;
        const voronoi_diagram<double>::edge_type* edge = it->incident_edge();
        vec2 vertex_pos = vec2(vertex.x(), vertex.y());
        vec2 p1;
        vec2 p2;
        do // This is convenient way to iterate edges around Voronoi vertex.
        {
            if (edge->is_primary())
            {
                if (edge->is_finite())
                {
                    p1 = vec2(edge->vertex1()->x(), edge->vertex1()->y());

                    if (edge->rot_next()->is_finite()) p2 = vec2(edge->rot_next()->vertex1()->x(), edge->rot_next()->vertex1()->y());
                    else
                    {
                        Site p1t = m_sites_with_Damage[edge->rot_next()->cell()->source_index()];
                        Site p2t = m_sites_with_Damage[edge->rot_next()->twin()->cell()->source_index()];
                        int end_x = (p1t.y - p2t.y) * 2;
                        int end_y = (p2t.x - p1t.x) * 2;
                        p2 = vertex_pos + vec2(end_x, end_y);
                    }
                }
                else
                {
                    Site p1s = m_sites_with_Damage[edge->cell()->source_index()];
                    Site p2s = m_sites_with_Damage[edge->twin()->cell()->source_index()];
                    int end_xs = (p1s.y - p2s.y) * 2;
                    int end_ys = (p2s.x - p1s.x) * 2;
                    p1 = vertex_pos + vec2(end_xs, end_ys);

                    if (edge->rot_next()->is_finite()) p2 = vec2(edge->rot_next()->vertex1()->x(), edge->rot_next()->vertex1()->y());
                    else
                    {
                        Site p1t = m_sites_with_Damage[edge->rot_next()->cell()->source_index()];
                        Site p2t = m_sites_with_Damage[edge->rot_next()->twin()->cell()->source_index()];
                        int end_x = (p1t.y - p2t.y) * 2;
                        int end_y = (p2t.x - p1t.x) * 2;
                        p2 = vertex_pos + vec2(end_x, end_y);
                    }
                }
                //int angle = getAngelOfTwoVector(p1, p2, vertex_pos);
                int angle;
                vec2 v1 = normalize(vec2(p1.x - vertex_pos.x, p1.y - vertex_pos.y));
                vec2 v2 = normalize(vec2(p2.x - vertex_pos.x, p2.y - vertex_pos.y));
                double theta;
                double dotp = dot(v1, v2);
                theta = acos(dotp);

                angle = round(theta * 180.0 / CV_PI);
                Angle_garage[angle] += 1;
                total_numbers += 1;
            }
            edge = edge->rot_next();
        } while (edge != it->incident_edge());
    }

    std::vector<double> frequency(181, 0);
    frequency = Angle_garage;
    for (size_t i = 0; i < x.size(); i++) {
        x[i] = i;
        sum += (double)i * Angle_garage[i];
        frequency[i] /= total_numbers;
    }
    auto axes = CvPlot::makePlotAxes();
    axes.create<CvPlot::Series>(x, frequency, "-r");
    axes.title("The frequency distribution histogram of vertex angles");

    double mean = sum / total_numbers;
    double sq_sum = 0;
    for (size_t j = 0; j < x.size(); j++) {
        sq_sum += (pow(((double)j - mean), 2) * Angle_garage[j]);
    }
    double stdev = std::sqrt(sq_sum / total_numbers);

    axes.xLabel("Alpha (Degree)");
    axes.yLabel("Relative Frequency (%)");
    //axes.create<MyRect>();
    cv::Mat mat = axes.render(1000, 1500);
    std::string var1 = "Mean = " + std::to_string(mean);
    std::string var2 = "Standard Deviation = " + std::to_string(stdev);
    std::string var3 = "Total Vertices Number = " + std::to_string(total_numbers);
    cv::putText(mat, var1, cv::Point(250, 200), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat, var2, cv::Point(250, 250), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat, var3, cv::Point(250, 300), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);

    cv::imwrite("./Statistical_data/Damage_Statistical_data_angle.png", mat);
    OutPutExcel("Damage_Statistical_data_angle", frequency);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<int> l(400);
    std::vector<double> length_garage(400, 0);
    std::unordered_map<const boost::polygon::voronoi_edge<double>*, int> eeedge_garage;
    int maxlength = 0;
    int minlength = 1000;
    double length_total_numbers = 0;
    double length_sum = 0;

    for (voronoi_diagram<double>::const_edge_iterator it = m_Voronoi_Damage_Diagram.edges().begin(); it != m_Voronoi_Damage_Diagram.edges().end(); ++it)
    {
        const voronoi_diagram<double>::edge_type& edge = *it;
        if (edge.is_finite() && edge.is_primary() && edge.twin()->is_finite() && edge.twin()->is_primary())
        {
            if (eeedge_garage[&edge] == 0 && eeedge_garage[edge.twin()] == 0)
            {
                vec2 vertex0 = vec2(edge.vertex0()->x(), edge.vertex0()->y());
                vec2 vertex1 = vec2(edge.vertex1()->x(), edge.vertex1()->y());
                int nowlength = round(glm::length(vertex0 - vertex1));
                if (nowlength < 400)
                {
                    length_garage[nowlength] += 1;
                    length_total_numbers += 1;
                    if (length_garage[nowlength] > 5)
                        maxlength = std::max(maxlength, nowlength);
                    //if
                    minlength = std::min(minlength, nowlength);
                }
                eeedge_garage[&edge] += 1;
                eeedge_garage[edge.twin()] += 1;
            }
        }
    }
    std::vector<double> length_frequency(400, 0);
    length_frequency = length_garage;
    for (size_t i = 0; i < l.size(); i++) {
        l[i] = i;
        length_sum += (double)i * length_garage[i];
        length_frequency[i] /= length_total_numbers;
    }
    auto axes2 = CvPlot::makePlotAxes();
    axes2.create<CvPlot::Series>(l, length_frequency, "-r");
    axes2.title("The frequency distribution histogram of edge lengths");

    double length_mean = length_sum / length_total_numbers;
    double length_sq_sum = 0;
    for (size_t j = 0; j < l.size(); j++) {
        length_sq_sum += (pow(((double)j - length_mean), 2) * length_garage[j]);
    }
    double length_stdev = std::sqrt(length_sq_sum / length_total_numbers);

    axes2.xLabel("Length (Pixel Length)");
    axes2.yLabel("Relative Frequency (%)");
    //axes.create<MyRect>();
    cv::Mat mat2 = axes2.render(1000, 1500);
    std::string var21 = "Mean = " + std::to_string(length_mean);
    std::string var22 = "Standard Deviation = " + std::to_string(length_stdev);
    std::string var33 = "Maximum Effective Length = " + std::to_string(maxlength);
    std::string var4 = "Total Edges Number = " + std::to_string(length_total_numbers);
    std::string var5 = "Total Sites Number = " + std::to_string(m_sites_with_Damage.size());
    cv::putText(mat2, var21, cv::Point(800, 200), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var22, cv::Point(800, 250), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var33, cv::Point(800, 300), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var4, cv::Point(800, 350), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::putText(mat2, var5, cv::Point(800, 400), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, false);
    cv::imwrite("./Statistical_data/Damage_Statistical_data_length.png", mat2);
    OutPutExcel("Damage_Statistical_data_length", length_frequency);
}

void MyVoronoi::ClearMap()
{
    m_vmap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    m_vmap.setTo(255);
    m_rendermap = cv::Mat::zeros(m_row, m_column, CV_8UC3);
    m_rendermap.setTo(0);
    cv::imwrite("./CrackMaps/Damage_Holes.png", m_rendermap);

}

void MyVoronoi::EraseData()
{
    m_old_time = 0; // m_old_time set back to 0 whwn go back to time 0
    m_crackseeds.erase(m_crackseeds.begin(), m_crackseeds.end());
    m_garbage_vets.erase(m_garbage_vets.begin(), m_garbage_vets.end());
}

void MyVoronoi::DrawLine(Site p1, Site p2, cv::Mat m, int width)
{
    cv::Point2i p_history = p1;
    for (int q = 0; q < 5; q++) // there are 3 random joints between the two nodes line
    {
        cv::Point2i vector = q * (p2 - p1) / 5.0;
        cv::Point2i ortho_vector = cv::Point2i(vector.y, -vector.x) * 0.25;
        cv::Point2i pi1;
        if (q == 4) pi1 = p2;
        else pi1 = (p1 + vector) + (Random::Float() - 0.5) * ortho_vector;

        float lum = 0;
        int width = 2;
        cv::line(m, p_history , pi1, cv::Scalar(lum, lum, lum), width, 4);//+ cv::Point2i((Random::Float() - 0.5), (Random::Float() - 0.5))
        p_history = pi1;
    }
    //cv::line(m_vmap, p1, p2, cv::Scalar(0, 0, 0), 1, 4);
}

void MyVoronoi::SetDrawFlag(bool b)
{
    m_ifdrawscndcrack = b;
}

void MyVoronoi::DrawCrack()
{
    m_max_width = -1;
    m_max_depth = -1;
    m_min_depth = 100000;
    m_min_width = 100000;
    m_quad_garage.clear();
    if (m_ifdrawscndcrack) {
        for (auto c : m_crackseeds)
        {
            if (c->Return_Children().size() != 0)
            {
                for (auto child : c->Return_Children())
                    DrawQuad(c, child);
            }
        }
    }
    else
    {
        for (int i = 0; i < m_scndcrackseedindex; i++)
        {
            Node* c = m_crackseeds[i];
            if (c->Return_Children().size() != 0)
            {
                for (auto child : c->Return_Children())
                    DrawQuad(c, child);
            }
        }
    }
}

double width_mul = 0.2;//cement: 0.2;//pitch:0.2//origin:2;
double scale_mul = 0.1;//porcelin:0.01//tree wheel:0.2 //cement:0.03; //pitch:0.05//mud:0.15,pitch:0.2;//

void MyVoronoi::DrawQuad(Node* node1, Node* node2)
{

    //  1---------------4
    //  |               |
    //  N1(p)           N2(c)   [ccw]
    //  |               |
    //  2---------------3

    if (node1->ReturnParent() == NULL && node2->Return_Children().size() != 0)
    {
        //if (m_old_time + 1 < node1->Return_Year())
        //    node1->m_depth = width_mul;
        //else
            node1->m_depth = (m_old_time - node1->Return_Year() + 1) * scale_mul + width_mul;

        node1->m_width = node1->m_depth;
        m_max_width = std::max(m_max_width, node1->m_width);
        m_max_depth = std::max(m_max_depth, node1->m_depth);
        m_min_width = std::min(m_min_width, node1->m_width);
        m_min_depth = std::min(m_min_depth, node1->m_depth);

        //if (m_old_time + 1 < node2->Return_Year())
        //    node2->m_depth = width_mul;
        //else
            node2->m_depth = (m_old_time - node2->Return_Year() + 1) * scale_mul + width_mul;
        node2->m_width = node2->m_depth;
        m_max_width = std::max(m_max_width, node2->m_width);
        m_max_depth = std::max(m_max_depth, node2->m_depth);
        m_min_width = std::min(m_min_width, node2->m_width);
        m_min_depth = std::min(m_min_depth, node2->m_depth);

        double N1x = node1->m_pos.x;
        double N1y = node1->m_pos.y;
        double N2x = node2->m_pos.x;
        double N2y = node2->m_pos.y;
        vec2 edge_normal = normalize(vec2((N1y - N2y), (N2x - N1x)));

        QuadNode N1 = QuadNode(N1x, N1y, node1->m_width, node1->m_depth, 0.0);
        QuadNode N2 = QuadNode(N2x, N2y, node2->m_width, node2->m_depth, 0.0);

        std::vector<QuadNode> quad;
        QuadNode p1 = QuadNode(
            N1x + edge_normal.x * 0.5 * node1->m_width,
            N1y + edge_normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);
        QuadNode p2 = QuadNode(
            N1x - edge_normal.x * 0.5 * node1->m_width,
            N1y - edge_normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);

        int N3x = node2->Return_Children()[0]->m_pos.x;
        int N3y = node2->Return_Children()[0]->m_pos.y;
        vec2 edge2_normal = normalize(vec2((N2y - N3y), (N3x - N2x)));
        vec2 n2_final_normal = normalize(edge_normal + edge2_normal);
        node2->normal = n2_final_normal;

        QuadNode p3 = QuadNode(
            N2x - node2->normal.x * 0.5 * node2->m_width,
            N2y - node2->normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);
        QuadNode p4 = QuadNode(
            N2x + node2->normal.x * 0.5 * node2->m_width,
            N2y + node2->normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);

        quad.push_back(p1); quad.push_back(N1); quad.push_back(p2);
        quad.push_back(p3); quad.push_back(N2); quad.push_back(p4);
        m_quad_garage.push_back(quad);

        for (auto node_child: node2->Return_Children())
            DrawQuad(node2, node_child);
    }
    else if (node1->ReturnParent() == NULL && node2->Return_Children().size() == 0)
    {
        //if (m_old_time + 1 < node1->Return_Year())
        //    node1->m_depth = width_mul;
        //else
            node1->m_depth = (m_old_time - node1->Return_Year() + 1) * scale_mul + width_mul;
        node1->m_width = node1->m_depth;
        m_max_width = std::max(m_max_width, node1->m_width);
        m_max_depth = std::max(m_max_depth, node1->m_depth);
        m_min_width = std::min(m_min_width, node1->m_width);
        m_min_depth = std::min(m_min_depth, node1->m_depth);

        //if (m_old_time + 1 < node2->Return_Year())
        //    node2->m_depth = width_mul;
        //else
            node2->m_depth = (m_old_time - node2->Return_Year() + 1) * scale_mul + width_mul;
        node2->m_width = node2->m_depth;
        m_max_width = std::max(m_max_width, node2->m_width);
        m_max_depth = std::max(m_max_depth, node2->m_depth);
        m_min_width = std::min(m_min_width, node2->m_width);
        m_min_depth = std::min(m_min_depth, node2->m_depth);


        double N1x = node1->m_pos.x;
        double N1y = node1->m_pos.y;
        double N2x = node2->m_pos.x;
        double N2y = node2->m_pos.y;
        vec2 edge_normal = normalize(vec2((N1y - N2y), (N2x - N1x)));

        QuadNode N1 = QuadNode(N1x, N1y, node1->m_width, node1->m_depth, 0.0);
        QuadNode N2 = QuadNode(N2x, N2y, node2->m_width, node2->m_depth, 0.0);

        std::vector<QuadNode> quad;
        QuadNode p1 = QuadNode(
            N1x + edge_normal.x * 0.5 * node1->m_width,
            N1y + edge_normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);
        QuadNode p2 = QuadNode(
            N1x - edge_normal.x * 0.5 * node1->m_width,
            N1y - edge_normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);

        QuadNode p3 = QuadNode(
            N2x - edge_normal.x * 0.5 * node2->m_width,
            N2y - edge_normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);
        QuadNode p4 = QuadNode(
            N2x + edge_normal.x * 0.5 * node2->m_width,
            N2y + edge_normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);

        quad.push_back(p1); quad.push_back(N1); quad.push_back(p2);
        quad.push_back(p3); quad.push_back(N2); quad.push_back(p4);
        m_quad_garage.push_back(quad);

        return;
    }
    else if (node1->ReturnParent() != NULL && node2->Return_Children().size() == 0)
    {
       //if (m_old_time + 1 < node2->Return_Year())
       //    node2->m_depth = width_mul;
       //else
            node2->m_depth = (m_old_time - node2->Return_Year() + 1) * scale_mul + width_mul;
        node2->m_width = node2->m_depth;
        m_max_width = std::max(m_max_width, node2->m_width);
        m_max_depth = std::max(m_max_depth, node2->m_depth);
        m_min_width = std::min(m_min_width, node2->m_width);
        m_min_depth = std::min(m_min_depth, node2->m_depth);

        double N1x = node1->m_pos.x;
        double N1y = node1->m_pos.y;
        double N2x = node2->m_pos.x;
        double N2y = node2->m_pos.y;
        vec2 edge_normal = normalize(vec2((N1y - N2y), (N2x - N1x)));

        QuadNode N1 = QuadNode(N1x, N1y, node1->m_width, node1->m_depth, 0.0);
        QuadNode N2 = QuadNode(N2x, N2y, node2->m_width, node2->m_depth, 0.0);

        std::vector<QuadNode> quad;
        QuadNode p1 = QuadNode(
            N1x + node1->normal.x * 0.5 * node1->m_width,
            N1y + node1->normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);
        QuadNode p2 = QuadNode(
            N1x - node1->normal.x * 0.5 * node1->m_width,
            N1y - node1->normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);

        QuadNode p3 = QuadNode(
            N2x - edge_normal.x * 0.5 * node2->m_width,
            N2y - edge_normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);
        QuadNode p4 = QuadNode(
            N2x + edge_normal.x * 0.5 * node2->m_width,
            N2y + edge_normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);

        quad.push_back(p1); quad.push_back(N1); quad.push_back(p2);
        quad.push_back(p3); quad.push_back(N2); quad.push_back(p4);
        m_quad_garage.push_back(quad);

        return;

    }
    else
    {
        //if (m_old_time + 1 < node2->Return_Year())
        //    node2->m_depth = width_mul;
        //else
            node2->m_depth = (m_old_time - node2->Return_Year() + 1) * scale_mul + width_mul;
        node2->m_width = node2->m_depth;
        m_max_width = std::max(m_max_width, node2->m_width);
        m_max_depth = std::max(m_max_depth, node2->m_depth);
        m_min_width = std::min(m_min_width, node2->m_width);
        m_min_depth = std::min(m_min_depth, node2->m_depth);

        double N1x = node1->m_pos.x;
        double N1y = node1->m_pos.y;
        double N2x = node2->m_pos.x;
        double N2y = node2->m_pos.y;
        vec2 edge_normal = normalize(vec2((N1y - N2y), (N2x - N1x)));

        QuadNode N1 = QuadNode(N1x, N1y, node1->m_width, node1->m_depth, 0.0);
        QuadNode N2 = QuadNode(N2x, N2y, node2->m_width, node2->m_depth, 0.0);

        std::vector<QuadNode> quad;
        QuadNode p1 = QuadNode(
            N1x + node1->normal.x * 0.5 * node1->m_width,
            N1y + node1->normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);
        QuadNode p2 = QuadNode(
            N1x - node1->normal.x * 0.5 * node1->m_width,
            N1y - node1->normal.y * 0.5 * node1->m_width,
            node1->m_width, node1->m_depth, 1.0);

        int N3x = node2->Return_Children()[0]->m_pos.x;
        int N3y = node2->Return_Children()[0]->m_pos.y;
        vec2 edge2_normal = normalize(vec2((N2y - N3y), (N3x - N2x)));
        vec2 n2_final_normal = normalize(edge_normal + edge2_normal);
        node2->normal = n2_final_normal;

        QuadNode p3 = QuadNode(
            N2x - node2->normal.x * 0.5 * node2->m_width,
            N2y - node2->normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);
        QuadNode p4 = QuadNode(
            N2x + node2->normal.x * 0.5 * node2->m_width,
            N2y + node2->normal.y * 0.5 * node2->m_width,
            node2->m_width, node2->m_depth, 1.0);

        quad.push_back(p1); quad.push_back(N1); quad.push_back(p2);
        quad.push_back(p3); quad.push_back(N2); quad.push_back(p4);
        m_quad_garage.push_back(quad);

        for (auto node_child : node2->Return_Children())
            DrawQuad(node2, node_child);
    }
}

void MyVoronoi::DrawLine(Node* node1, Node* node2, int line_level)
{
    Site p1 = cv::Point2i(node1->m_pos.x, node1->m_pos.y);
    Site p2 = cv::Point2i(node2->m_pos.x, node2->m_pos.y);
    double length = cv::norm(p1 - p2);//sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));

    int cousin_flag = false;
    if (length > 100 || line_level != -1)
        cousin_flag = true;

    //if (cousin_flag)
    //{
    //    cv::Point2i ortho = cv::Point2i((p1 - p2).y, -(p1 - p2).x);
    //    ortho = ortho/cv::norm(ortho)*7;
    //    cv::line(m_vmap, p1 + ortho, p2 + ortho, cv::Scalar(10, 0, 0), 1, 4);
    //}


    int iterate_times = length * 9.0 / 50.0 + 1;
    cv::Point2i p_history = p1;
    for (int q = 0; q < iterate_times; q++) // there are 3 random joints between the two nodes line
    {
        cv::Point2i vector = q * (p2 - p1) / (iterate_times+1);
        cv::Point2i ortho_vector = cv::Point2i(vector.y, -vector.x);
        int amplitude = 6; // the maximum jittering distance is: 7
        if (vector.x == 0 && vector.y == 0)
            ortho_vector = cv::Point2i(0, 0);
        else
            ortho_vector = ortho_vector/cv::norm(ortho_vector) * amplitude;
        cv::Point2i pi1;
        if (q == iterate_times - 1) pi1 = p2;
        else pi1 = (p1 + vector) + (Random::Float() - 0.5) * ortho_vector;
    
        float lum = 10 + Random::Float()*30;
        int width;
        if (cousin_flag) width = (2*Random::Float())+1;
        else width = Random::Float() + 1;

        //if (cousin_flag)
        //{
        //    float lum1 = 10 + Random::Float() * 30;
        //    cv::Point2i ortho = cv::Point2i((p_history - pi1).y, -(p_history - pi1).x);
        //    ortho = ortho / cv::norm(ortho) * 7;
        //    cv::line(m_vmap, p_history + ortho, pi1 + ortho, cv::Scalar(lum1, lum1, lum1), 1, 4);
        //}
        //else
            cv::line(m_vmap, p_history, pi1, cv::Scalar(lum, lum, lum), width, 4);//+ cv::Point2i((Random::Float() - 0.5), (Random::Float() - 0.5))
        p_history = pi1;
    }
    //cv::line(m_vmap, p1, p2, cv::Scalar(0, 0, 0), 1, 4);
}

//double MyVoronoi::CalculateArea(const voronoi_diagram<double>::cell_type* cell)
//{
//    double Area = 0;
//    const voronoi_diagram<double>::edge_type* edge = cell->incident_edge();
//    do {
//        if (edge->is_infinite())
//        {
//            qWarning() << "Find a endless cell located in: " << m_sites_with_Damage[cell->source_index()].x << ", " << m_sites_with_Damage[cell->source_index()].y;
//            return 1;
//        }
//        Area += (edge->vertex0()->x() * edge->vertex1()->y() - edge->vertex1()->x() * edge->vertex0()->y());
//        edge = edge->next();
//        // Do smth. with edge.
//    } while (edge != cell->incident_edge());
//    qDebug() << "Area of cell" << cell->source_index() << "is: " << 0.5 * Area;
//    return 0.5 * Area;
//}

double MyVoronoi::CalculateArea(HVD::cell* cell)
{
    double Area = 0;
    for (auto edge : cell->m_edges)
    {
        if (edge->m_infinit_vert == vec2(-1,-1))
        {
            qWarning() << "Find a endless cell located in: " ;
            return 1;
        }
        Area += (edge->m_verts[0]->m_pos.x * edge->m_verts[1]->m_pos.y - edge->m_verts[1]->m_pos.x * edge->m_verts[0]->m_pos.y);

    }
    qDebug() << "Area of cell" << "xx" << "is: " << 0.5 * Area; //cell->m_index()
    return 0.5 * Area;
}

cv::Mat MyVoronoi::ShowMap()
{
    return m_vmap;
}


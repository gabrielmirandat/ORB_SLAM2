X// Converter - converter entre matrizes, vetores, Mats, quaternion, SE3
X// Frame - calcula BoW, extrai ORB, checa se mapPoint tá no frustrum da camera, obtem features numa area, converte features para grid para facilitar calculos
X// FrameDrawer - desenha na tela
X// Initializer - funcões para inicialização, computar matrizes fundamental, de homografia, essencial, checar limiares
X// KeyFrame - ComputeBoW, funções do grafo de covisibilidade, funções de spanning tree, manipular pontos do mapa, features em área
X// KeyFrameDatabase - banco de keyframes comum, métodos DetectLoopCandidates, DetectRelocalizationCandidates
X// LocalMapping - realiza mapeamento numa thread separada do tracking, checa novos keyframes e faz culling dos parecidos
// LoopClosing - realiza global bundle adjustment e compute sim3, corrige mapa também
X// Map - armazena info do mapa
X// MapDrawer - desenha mapa usando Pangolin OpenGL
X// MapPoint - representa um ponto do mapa, posição global, número de observações e keyframes observados, prediz escala do ponto no mapa
X// Optimizer - realiza bundle adjustment, local e global, otimização de pose, otimização de grafo, otimização de sim3
X// ORBextractor - metodo de extração e assinatura de features orb, parametros obtidos do arquivo de calibracao, métodos ComputePyramid, ComputeKeyPointsOctTree, ComputeKeyPointsOld, DivideNode
X// ORBmatcher - calcula distância de hamming, realiza projeções entre frames, keyframes, ambos, pontos 3D no mapa, pelo bag of words, atua na inicialização, triangulação, Sim3
X// ORBVocabulary - só um typedef
X// PnPsolver - resolve o problema de encontrar pose 6DoF dados n pontos 3D com suas respectivas projeções 2D, usa RANSAC para remover outliers, usa Gaus-Newton para refinar coeficientes Beta
X// Sim3Solver - resolve o Sim3 da algebra de Lie, obtém o 7DoF
X// System - inicia sistema e chama todas as threads
X// Tracking - mais complexo, vide abaixo
X// Viewer - desenha na tela usando Pangolin OpenGL


Bow - bag of words
# histograma de features ORB da imagem para fácil checagem e comparação

SE3 - The Lie Group SE(3) // PARA STEREO E RGBD APENAS ?????
# 3D Rigid transformations
# 6 Dimensions
# Linear transformation on homogeneous 4-vectors
# mathematical basis of rigid body transformations and velocities
# every such transformation is the combination of a translation and a (proper) rotation, x′=Rx+c, where R is a rotation matrix and c is a 3d vector
# the set of all transformations that can be applied to a rigid body, plus the operations of composing and inverting them
# The motion of a rigid body can be expressed as a curve in SE(3) which gives the transformation from the world to the body frame for each instant in time

g2o::Sim3 - The Lie Group Sim(3) // PARA MONOCULAR APENAS ????
                                 // PROBLEMA DA ESCALA ????
# 3D Similarity transformations (rigid motion + scale)
# 7 Dimensions
# Linear transformation on homogeneous 4-vectors
# pose graph optimization
# algoritmo de otimização de grafo de poses
# transformação de similaridade

PnP - Perspective N Point
# problema de estimar a pose de camera calibrada dado um set de "n" pontos 3D do mundo e suas correspondendes projeções 2D na imagem. A pose estimada consiste de 6 degrees-of-freedom (DOF) feitos de rotação (roll, pitch, yaw) e translação (x, y, z). Uma solução comum existe para n=3, o P3P, e muitas soluções existem para o caso geral, n >=3.
#  follows the perspective project model for cameras: K[R|T]pw
# PnP is prone to errors if there are outliers in the set of point correspondences. Thus, RANSAC can be used in conjunction with existing solutions to make the final solution for the camera pose more robust to outliers.

H - homography matrix
F - fundamental matrix
E - essential matrix


// Number of KeyPoints
const int N; // Frame::N

------------------------------------------------------------------------TRACKING

//ORB
ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
ORBextractor* mpIniORBextractor;

// Initalization (only for monocular)
Initializer* mpInitializer;

//Map
Map* mpMap;

//BoW
ORBVocabulary* mpORBVocabulary;
KeyFrameDatabase* mpKeyFrameDB;

//Motion Model
cv::Mat mVelocity;

//Calibration matrix
cv::Mat mK;
cv::Mat mDistCoef;
float mbf;

//New KeyFrame rules (according to fps)
int mMinFrames;
int mMaxFrames;

//Current matches in frame
int mnMatchesInliers;

// Main tracking function. It is independent of the input sensor.
void Track();

// Map initialization for monocular
void MonocularInitialization();
void CreateInitialMapMonocular();

bool TrackWithMotionModel();

// Preprocess the input and call Track(). Extract features and performs stereo matching.
cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

// Load ORB parameters
int nFeatures = fSettings["ORBextractor.nFeatures"];
float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
int nLevels = fSettings["ORBextractor.nLevels"];
int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
int fMinThFAST = fSettings["ORBextractor.minThFAST"];

mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

if(sensor==System::STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

if(sensor==System::MONOCULAR)
    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

método Tracking::Track() // suma importancia

// Initialization
if(mState==NOT_INITIALIZED)
{
  if(mSensor==System::STEREO || mSensor==System::RGBD)
      StereoInitialization();
  else
      MonocularInitialization();
}
// Tracking
else
{
  // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
}

// Track with reference keyframe or with MotionModel!!!
if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
{
    bOK = TrackReferenceKeyFrame();
}
else
{
    bOK = TrackWithMotionModel();
    if(!bOK)
        bOK = TrackReferenceKeyFrame();
}

// If tracking were good, check if we insert a keyframe
if(bOK)
{
    // Update motion model
    if(!mLastFrame.mTcw.empty())
    {
        cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
        mVelocity = mCurrentFrame.mTcw*LastTwc;
    }
    else
        mVelocity = cv::Mat();

// We allow points with high innovation (considererd outliers by the Huber Function)
// pass to the new keyframe, so that bundle adjustment will finally decide
// if they are outliers or not. We don't want next frame to estimate its position
// with those points so we discard them in the frame.
for(int i=0; i<mCurrentFrame.N;i++)
{
    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
}

método Tracking::MonocularInitialization()

if(!mpInitializer)
{
    // Set Reference Frame
    if(mCurrentFrame.mvKeys.size()>100)
    {
        mInitialFrame = Frame(mCurrentFrame);
        mLastFrame = Frame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
        for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
            mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

        if(mpInitializer)
            delete mpInitializer;

        mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

        return;
    }
}
else
{
    ...CreateInitialMapMonocular();
}

método Tracking::CreateInitialMapMonocular()

// Create KeyFrames
KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


pKFini->ComputeBoW();
pKFcur->ComputeBoW();

// Insert KFs in the map
mpMap->AddKeyFrame(pKFini);
mpMap->AddKeyFrame(pKFcur);

// Update Connections
pKFini->UpdateConnections();
pKFcur->UpdateConnections();

// Bundle Adjustment
cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

Optimizer::GlobalBundleAdjustemnt(mpMap,20);

if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
{
    cout << "Wrong initialization, reseting..." << endl;
    Reset();
    return;
}

// já é o rastreamento propriamnete dito
// não é mais inicialização
método Tracking::TrackReferenceKeyFrame()

// Compute Bag of Words vector
mCurrentFrame.ComputeBoW();

// We perform first an ORB matching with the reference keyframe
// If enough matches are found we setup a PnP solver
ORBmatcher matcher(0.7,true);
vector<MapPoint*> vpMapPointMatches;

int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

if(nmatches<15)
    return false;

mCurrentFrame.mvpMapPoints = vpMapPointMatches;
mCurrentFrame.SetPose(mLastFrame.mTcw);

Optimizer::PoseOptimization(&mCurrentFrame);

método Tracking::TrackWithMotionModel()

ORBmatcher matcher(0.9,true);
// Update last frame pose according to its reference keyframe
// Create "visual odometry" points if in Localization Mode
UpdateLastFrame();

mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

... return nmatchesMap>=10;

// We have an estimation of the camera pose and some map points tracked in the frame.
// We retrieve the local map and try to find matches to points in the local map.
método Tracking::TrackLocalMap()

método Tracking::NeedNewKeyFrame()

// If Local Mapping is freezed by a Loop Closure do not insert keyframes
if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

// Thresholds
float thRefRatio = 0.75f;
if(nKFs<2)
    thRefRatio = 0.4f;

if(mSensor==System::MONOCULAR)
    thRefRatio = 0.9f;


método Tracking::SearchLocalPoints()
...

método Tracking::Relocalization()

// Compute Bag of Words Vector
mCurrentFrame.ComputeBoW();

// Relocalization is performed when tracking is lost
// Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

// We perform first an ORB matching with each candidate
// If enough matches are found we setup a PnP solver
ORBmatcher matcher(0.75,true);

vector<PnPsolver*> vpPnPsolvers;
vpPnPsolvers.resize(nKFs);

int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
if(nmatches<15)
{
    vbDiscarded[i] = true;
    continue;
}
else
{
    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
    vpPnPsolvers[i] = pSolver;
    nCandidates++;
}

// Alternatively perform some iterations of P4P RANSAC
// Until we found a camera pose supported by enough inliers
bool bMatch = false;
ORBmatcher matcher2(0.9,true);

// only monocular slam
---------------------------------------------------------------------INITIALIZER

// Keypoints from Reference Frame (Frame 1)
vector<cv::KeyPoint> mvKeys1;

// Keypoints from Current Frame (Frame 2)
vector<cv::KeyPoint> mvKeys2;

// Current Matches from Reference to Current
vector<Match> mvMatches12;
vector<bool> mvbMatched1;

// Calibration
cv::Mat mK;

// Standard Deviation and Variance
float mSigma, mSigma2;

// Ransac max iterations
int mMaxIterations;

método FindHomography()
método FindFundamenal()
...    ComputeH21() // homography between frame and reference frame
...    ComputeF21() // fundamental between frame and reference frame
ReconstructF(), ReconstructFH()
Triangulate(), DecomposeE(), // essecial matrix

// Generate sets of 8 points for each RANSAC iteration
mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

DUtils::Random::SeedRandOnce(0);

for(int it=0; it<mMaxIterations; it++)
{
    vAvailableIndices = vAllIndices;

    // Select a minimum set
    for(size_t j=0; j<8; j++)
    {
        int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
        int idx = vAvailableIndices[randi];

        mvSets[it][j] = idx;

        vAvailableIndices[randi] = vAvailableIndices.back();
        vAvailableIndices.pop_back();
    }
}

// Launch threads to compute in parallel a fundamental matrix and a homography
vector<bool> vbMatchesInliersH, vbMatchesInliersF;
float SH, SF;
cv::Mat H, F;

thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

// Wait until both threads have finished
threadH.join();
threadF.join();

// Compute ratio of scores
float RH = SH/(SH+SF);

// Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
if(RH>0.40)
    return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
else //if(pF_HF>0.6)
    return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

return false;

cv::Mat u,w,vt;
cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
return vt.row(8).reshape(0, 3);

cv::SVDecomp()

método ReconstructF()

int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

R21 = cv::Mat();
t21 = cv::Mat();

int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

int nsimilar = 0;
if(nGood1>0.7*maxGood)
    nsimilar++;
if(nGood2>0.7*maxGood)
    nsimilar++;
if(nGood3>0.7*maxGood)
    nsimilar++;
if(nGood4>0.7*maxGood)
    nsimilar++;

// If there is not a clear winner or not enough triangulated points reject initialization
if(maxGood<nMinGood || nsimilar>1)
{
    return false;
}

// If best reconstruction has enough parallax initialize
if(maxGood==nGood1)
{
    if(parallax1>minParallax)
    {
        vP3D = vP3D1;
        vbTriangulated = vbTriangulated1;

        R1.copyTo(R21);
        t1.copyTo(t21);
        return true;
    }
}else if(maxGood==nGood2)
{...

método ReconstructH()

// We recover 8 motion hypotheses using the method of Faugeras et al.
// Motion and structure from motion in a piecewise planar environment.
// International Journal of Pattern Recognition and Artificial Intelligence, 1988
cv::Mat invK = K.inv();
cv::Mat A = invK*H21*K;

cv::Mat U,w,Vt,V;
cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
V=Vt.t();

float s = cv::determinant(U)*cv::determinant(Vt);

float d1 = w.at<float>(0);
float d2 = w.at<float>(1);
float d3 = w.at<float>(2);

if(d1/d2<1.00001 || d2/d3<1.00001)
{
    return false;
}

// Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
// We reconstruct all hypotheses and check in terms of triangulated points and parallax
for(size_t i=0; i<8; i++)
{
    float parallaxi;
    vector<cv::Point3f> vP3Di;
    vector<bool> vbTriangulatedi;
    int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

    if(nGood>bestGood)
    {
        secondBestGood = bestGood;
        bestGood = nGood;
        bestSolutionIdx = i;
        bestParallax = parallaxi;
        bestP3D = vP3Di;
        bestTriangulated = vbTriangulatedi;
    }
    else if(nGood>secondBestGood)
    {
        secondBestGood = nGood;
    }
}
if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

método CheckRT()

// Camera 1 Projection Matrix K[I|0]
cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
K.copyTo(P1.rowRange(0,3).colRange(0,3));

cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

// Camera 2 Projection Matrix K[R|t]
cv::Mat P2(3,4,CV_32F);
R.copyTo(P2.rowRange(0,3).colRange(0,3));
t.copyTo(P2.rowRange(0,3).col(3));
P2 = K*P2;

for(size_t i=0, iend=vMatches12.size();i<iend;i++)
{
   Triangulate(kp1,kp2,P1,P2,p3dC1);
   // Check parallax
   cv::Mat normal1 = p3dC1 - O1;
   float dist1 = cv::norm(normal1);

   cv::Mat normal2 = p3dC1 - O2;
   float dist2 = cv::norm(normal2);

   float cosParallax = normal1.dot(normal2)/(dist1*dist2);
}

// Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
    continue;

// Check reprojection error in first image
float im1x, im1y;
float invZ1 = 1.0/p3dC1.at<float>(2);
im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

if(squareError1>th2)
    continue;

DecomposeE()

cv::Mat u,w,vt;
cv::SVD::compute(E,w,u,vt);

u.col(2).copyTo(t);
t=t/cv::norm(t);

cv::Mat W(3,3,CV_32F,cv::Scalar(0));
W.at<float>(0,1)=-1;
W.at<float>(1,0)=1;
W.at<float>(2,2)=1;

R1 = u*W*vt;
if(cv::determinant(R1)<0)
    R1=-R1;

R2 = u*W.t()*vt;
if(cv::determinant(R2)<0)
    R2=-R2;
}

--------------------------------------------------------------------LOCALMAPPING

método Run() principal
CheckNewKeyFrames(), ProcessNewKeyFrame(), CreateNewMapPoints(), KeyFrameCulling(), ComputeF12()

método run()

continuamente processa keyframes novos e faz mapeamento

---------------------------------------------------------------------LOOPCLOSING

float mnCovisibilityConsistencyTh;
g2o::Sim3 mg2oScw;

método Run() principal

if(CheckNewKeyFrames())
{
    // Detect loop candidates and check covisibility consistency
    if(DetectLoop())
    {
       // Compute similarity transformation [sR|t]
       // In the stereo/RGBD case s=1
       if(ComputeSim3())
       {
           // Perform loop fusion and pose graph optimization
           CorrectLoop();


// This function will run in a separate thread
// Update all MapPoints and KeyFrames
// Local Mapping was active during BA, that means that there might be new keyframes
// not included in the Global BA and they are not consistent with the updated map.
// We need to propagate the correction through the spanning tree
método void RunGlobalBundleAdjustment(unsigned long nLoopKF)
  Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);


método DetectLoop()
// For each consistent loop candidate we try to compute a Sim3
// Perform alternatively RANSAC iterations for each candidate
// until one is succesful or all fail
método ComputeSim3()
método CorrectLoop()
  Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

-----------------------------------------------------------------------OPTIMIZER
COMPLICADO!!

static BundleAdjustment()
  g2o::SparseOptimizer
  g2o::BlockSolver_6_3
  g2o::LinearSolverEigen
  g2o::BlockSolver_6_3
  g2o::OptimizationAlgorithmLevenberg
  g2o::EdgeSE3ProjectXYZ
  g2o::RobustKernelHuber
  g2o::EdgeStereoSE3ProjectXYZ
  g2o::RobustKernelHuber
  g2o::VertexSE3Expmap
  g2o::SE3Quat
  g2o::VertexSBAPointXYZ

static GlobalBundleAdjustemnt()
static LocalBundleAdjustment()
static PoseOptimization()
  - parecido com BundleAdjustment

// if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
// if scale is fix, use 6DoF (stereo, rgbd), else 7DoF (monocular)
static OptimizeEssentialGraph()

// if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
// if scale is fix, optimize SE3, else Sim3 (monocular)
static OptimizeSim3()

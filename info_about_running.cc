initialization
  (HOMOGRAFIA - cena planar) x (FUNDAMENTAL - cena complexa)

https://github.com/raulmur/ORB_SLAM2/issues/59

  I would suggest to turn off the 'auto-focus' of your (smartphone?)-camera and set it to 'fixed/infinity'
  With this fixed af you have to do your 'camera calibration again 'and also keep it fixed when making new videos
  To reduce the motion blur, either try to set your 'exposure time to a lower value' or simply make your test video in a 'brighter environment'
  ORB_SLAM - no matter what version - works best with 'global shutter cameras', whereas your camera surely has rolling shutter

  increased ORBextractor.nFeatures to 2000

  Could you try this way:
  At Initializer::checkRT : 890, put the nGood inside the if{}:
  if(cosParallax<0.99998){
    vbGood[vMatches12[i].first]=true;
    nGood++;
  }

  It doesnt initialize with a planar surface which is required for my application. There is not enough depth in the scene to initialize with the fundamental matrix. The condition "secondBestGood<0.75*bestGood" in Initializer::ReconstructH is never satisfied.

  DISSERAM QUE EXISTEM DOIS BUGS NAO RESOLVIDOS!!

    #1
    At Initializer::checkRT : 890, put the nGood inside the if{}:
    if(cosParallax<0.99998){
      vbGood[vMatches12[i].first]=true;
      **_nGood++;_**
    }

    #2
    At ORBmatcher::SearchForInitialization: 485, correct bin value
    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
    if(rot<0.0)
        rot+=360.0f;
    **_int bin = round(rot/(360.0f*factor));_**
    if(bin==HISTO_LENGTH)
        bin=0;

https://github.com/raulmur/ORB_SLAM2/issues/66

  the initialization is pretty fast in a complex scene

  The parallax depends the degree of the camera to that planar. If the Z-axis is nearly parallel to the test plane, the little parallax should win (because most points are quite far). Instead, if the z-axis is perpendicular to the test plane, the bigger parallax wins

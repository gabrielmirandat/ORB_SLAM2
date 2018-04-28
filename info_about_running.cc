CONSIDERAÇÔES
  # proximo video
    - andar com 0.3 de vel e virar sempre bem devagar
    - ao virar ou passar por faixa de deficiente, andar beeem devagar e girar beeem devagar (passar pela faixa e girar rápido perde tracking)
    - colocar elementos na cena pro fundo não ficar todo branco, lixos são uma boa, mas não deixar vazio!!


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

https://github.com/raulmur/ORB_SLAM2/issues/194

  Initialization is random when you have few opportunities to initialize. When you record de video, aim under the horizon so you see the floor near you, and move laterally. The different points velocity help to triangulate (this is an over simplificated argument)

  Orb-slam has two initialization algorithm, one for volumetric features and another for coplanar features. They are the best algorithms available, sort of. Coplanar initialization tends to be more difficult in my experience

rotation

https://github.com/raulmur/ORB_SLAM2/issues/194

  The solution is not in the algorithm, but in the video, remaking it with some considerations in mind

  New map points are created via triangulation: so, you need frames taken from different positions, rotation only doesnt achieve that

  they not provide enough info to triangulate new map points, and the scene change very fast, faster than translations

  When tracking, dont suddenly rotate to new areas, rotate slowly and move simultaneously. orb-slam2 need some time to make new keyframes and add new points to the map. It depends heavyly on your processing power. With low power slowing down the video helps

  In difficult places (tracking loss) you can go forwards and backwards while playing your video and see orb.slam adding more keyframes and therefore more map points in the same area, that help tracking to the next new area

  For triangulate a point you need to observe it from two different places, say two different positions

scale after lose track

  The scale issue is normal in Mono camera, because for mono camera (or said sfm), the reconstruction matrix can only be back to scale level.

  Of course we have optimization tools such as SIM3 and BA, but there is no guarantee. Because the scale change, especially after tracking losing, has correct physical and numerical meaning (that's to say you can't not tell the new scale is right or wrong).

  However, if you have some prior knowledge or be sure that scale will never change, you can use looping BA when it recognize a loop, as in ORB-SLAM2.

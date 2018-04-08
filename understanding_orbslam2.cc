// LBA - local bundle adjustment
// FBA - full bundle adjustment
// PGO - pose graph otimization

# ros_mono.cc
	Classe ImageGrabber
	// obtém imagens
		GrabImage
		// subscriber de /camera/image_raw
		// chama System::TrackMonocular passando imagens
	main
	// chama System::construtor para iniciar todas as threads
	// para todas as threads
	// desliga sistema

# classe System
// chama todos os módulos
// vocabulário ORB para `place recognition`
// banco de keyframes para `place recognition, relocalization, loop detection`
// mapa que armazena ponteiros de `keyframes e mappoints`
// tracker para calcular poses de frames, decidir quando inserir keyframes, criar `mappoints` e fazer `relocalization` se perder tracking
// mapa local para gerenciar mapa local e `LBA`
// fechador de loop que busca loops a cada keyframe, faz PGO e em seguida FBA ao achar loop
// viewers do mapa e keyframes
// 4 threads `tracking (main), local mapping, loop closing, viewer`
// flag de reset, modo localização, estado do `traking`
	ctr
	// inicia threads restantes `local mapping, loop closing, viewer`
	// carrega arquivos de entrada, vocabulário ORB e calibração
	// cria database de `keyframes` com vocabulário ORB
	TrackStereo, TrackRGBD, TrackMonocular
	// processa frame monocular, converte para `grayscale`, retorna pose ou vazio se `tracking` falhar
	// checa mudanças de modo e reset
	ActivateLocalizationMode, DeactivateLocalizationMode
	// para/retorna thread `local mapping` para realizar só o `tracking`
	MapChanged
	// checa se mapa teve grande mudança com `loop closing`
	Reset, Shutdown
	// reseta/termina sistema, terminando todas as threads
	SaveTrajectoryTUM, SaveKeyFrameTrajectoryTUM, SaveTrajectoryKITTI
	// salva trajetória estéreo/RGBD da camera no TUM/KITTI, poses de `keyframes` estéreo/RGBD/monocular no TUM
	GetTrackingState, GetTrackedMapPoints, GetTrackedKeyPointsUn
	// informações do `tracking` do frame mais recente


# classe Tracker
// 
// 
// 
// 
//  

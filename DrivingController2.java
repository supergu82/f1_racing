public class DrivingController2 {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};

	public DrivingCmd controlDriving(double[] driveArray, double[] aicarArray, double[] trackArray, double[] damageArray, int[] rankArray, int trackCurveType, double[] trackAngleArray, double[] trackDistArray, double trackCurrentAngle){
		DrivingCmd cmd = new DrivingCmd();
		
		////////////////////// input parameters
		double toMiddle     = driveArray[DrivingInterface.drvie_toMiddle    ];
		double angle        = driveArray[DrivingInterface.drvie_angle       ];
		double speed        = driveArray[DrivingInterface.drvie_speed       ];

		double toStart				 = trackArray[DrivingInterface.track_toStart		];
		double dist_track			 = trackArray[DrivingInterface.track_dist_track		];
		double track_width			 = trackArray[DrivingInterface.track_width			];
		double track_dist_straight	 = trackArray[DrivingInterface.track_dist_straight	];
		int track_curve_type		 = trackCurveType;

		double[] track_forward_angles	= trackAngleArray;
		double[] track_forward_dists	= trackDistArray;
		double track_current_angle		= trackCurrentAngle;
		
		double[] dist_cars = aicarArray;
		
		double damage		 = damageArray[DrivingInterface.damage];
		double damage_max	 = damageArray[DrivingInterface.damage_max];

		int total_car_num	 = rankArray[DrivingInterface.rank_total_car_num	];
		int my_rank			 = rankArray[DrivingInterface.rank_my_rank			];
		int opponent_rank	 = rankArray[DrivingInterface.rank_opponent_rank	];		
		////////////////////// END input parameters
		
		
		
		System.out.println("======================== start ================================");
//		System.out.println("track_curve_type       : " + ( (track_curve_type==1)?"우회전":"좌회전"));
//		System.out.println("track_width            = " + track_width);	
//		System.out.println("toMiddle               = " + toMiddle);
//		System.out.println("angle                  = " + angle);
//		System.out.println("track_dist_straight    : " + track_dist_straight);
		
		/* user input parameters */
		double user_steer_coeff = 0.5; // 핸들계수(트랙정보에 따라 셋팅)
		double user_best_speed  = 100;
		double[] user_change_dist = new double[2];
		double user_steerCtl    = 0.0;
		double user_brakeCtl    = 0.0;
		double user_accelCtl    = 0.0;
		int user_backwordCtl    = DrivingInterface.gear_type_forward;	
		
		// 최적 이동 함수 
		user_change_dist[0] = this.getBestDist(toMiddle, track_width, track_curve_type);
		
//		System.out.println("user_change_dist opti : " + user_change_dist[0]);
		// 장애물 처리 함수
		user_change_dist = this.getChangeDist(dist_cars, toMiddle, user_change_dist[0], speed, track_width, track_dist_straight, track_curve_type);
						
		// 방향 제어 함수
		user_steerCtl = this.getSteerCtl(angle, user_change_dist[0], track_width, user_steer_coeff);
		
		// 속도 제어 함수
		user_best_speed = this.getBestSpeed(angle, user_change_dist[1], speed, track_dist_straight, track_curve_type);
		
		// 브레이크, 엑셀 제어
		double[] user_speed_ctl = this.getSpeedCtl(speed, user_best_speed, track_dist_straight);
		
		user_accelCtl = user_speed_ctl[0];
		user_brakeCtl = user_speed_ctl[1];
		
		System.out.println("======================== end ================================");
		
		////////////////////// output values		
		cmd.steer    = user_steerCtl;
		cmd.accel    = user_accelCtl;
		cmd.brake    = user_brakeCtl;
		cmd.backward = user_backwordCtl;
		////////////////////// END output values
		
		return cmd;		
	}
	


	/**
	 * 장애물 처리 함수
	 * @param curr_aicars
	 * @param change_toMiddle
	 * @param curr_speed
	 * @param curr_track_width
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getChangeDist(double[] curr_aicars, double curr_toMiddle, double change_toMiddle, double curr_speed, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		

		
		double user_check_forward = 100;
		double user_car_width = 2; // 전폭 2m
		double user_car_width_half = user_car_width/2; // 전폭 2m
		double ai_car_length      = 4.8; // 전장 4.8
		double ai_car_length_half = ai_car_length/2; // 전장 4.8
		
		if(curr_speed < 70){
			user_check_forward = 50;
		}else if(curr_speed < 90 && curr_speed >=70){
			user_check_forward = 70;
		}else{
			user_check_forward = 100;
		}		
		
		/*테스트 셋팅 */
		user_check_forward = 20;
		/*테스트 셋팅 */

		
		double[] user_change_dist = new double[2]; // change_toMiddle;
		user_change_dist[0] = change_toMiddle; // 변경할 거리
		user_change_dist[1] = 100; // 전방 장애물 거리
		// 전방 차량 모두 검증
		double[] cars_dist = new double[2];
		for(int i=0;i<5;i++){
//			System.out.println("dist_cars_dist[" + i + "]    = " + curr_aicars[2*i]);
//			System.out.println("dist_cars_middle[" + i + "]  = " + curr_aicars[2*i + 1]);
			if(i == 0 && curr_aicars[2*i] == 100){
//				System.out.println("전방 장애물차량 없음");
				break;
			}else{
				if(curr_aicars[2*i] == 100) {
					continue;
				}
			}
			
			cars_dist = this.getCarsDist(curr_toMiddle, change_toMiddle, curr_aicars[2*i + 1], curr_track_curve_type, user_car_width, ai_car_length);
			
			if(cars_dist[0] > 0.5){
//				System.out.println("전방 차량[" + i + "] 번 무시");
				continue;
			}
			
			if(cars_dist[0] > 0 && cars_dist[0] <= 0.5){
//				System.out.println("좌우 차량[" + i + "] 확인 필요");
			}
			
			if(cars_dist[0] <= 0){
//				System.out.println("전방 차량[" + i + "] 번 회피");
				user_change_dist[0] = cars_dist[1];
				user_change_dist[1] = curr_aicars[2*i];
				break;
			}
		}
		
/*		
		
		
		
		// 전방 첫번째 차량
		double curr_aicoars_01_dist   = curr_aicars[0];
		double curr_aicoars_01_middle = curr_aicars[1];
		double curr_01_middle_dist   = 0;
		double curr_01_dist   = 0;
		// 후방 첫번째 차량
		double curr_aicoars_10_dist   = curr_aicars[10];
		double curr_aicoars_10_middle = curr_aicars[11];
		double curr_10_middle_dist   = 0;
		double curr_10_dist   = 0;
		
		
		
		
		if(curr_aicoars_01_dist > 5 && curr_aicoars_01_dist < user_check_forward){
			System.out.println("+++++++++++++++++ 장애물 처리 함수[start] ++++++++++++++++++++++");
			System.out.println("track_curve_type : " + ( (curr_track_curve_type==1)?"우회전":"좌회전"));
			System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			System.out.println("curr_aicoars_01_dist     = "+curr_aicoars_01_dist);
			System.out.println("curr_aicoars_01_middle   = "+curr_aicoars_01_middle);
			System.out.println("change_toMiddle          = "+change_toMiddle);
			System.out.println("전방 "+user_check_forward+"m 이내 전방차량  존재하니 로직 준비 하세요.");
			
			
			curr_01_middle_dist =  curr_toMiddle - curr_aicoars_01_middle; // 첫번째 전방 차량과의 중심 거리
			curr_10_middle_dist =  curr_toMiddle - curr_aicoars_10_middle; // 첫번째 후방 차량과의 중심 거리
			
			// 전방 장애물 차량과 중심거리를 이용해 좌측(양수)에 있는지 우측(음수)에 있는지 판단 			
			this.getCarsDist(curr_toMiddle, change_toMiddle, curr_aicoars_01_middle, curr_track_curve_type, user_car_width, ai_car_length);
			
			System.out.println("장애물 middle 거리 (줌심거리)="+curr_01_middle_dist);
			System.out.println("장애물 거리 (실제거리)="+curr_01_dist);
			
			// 후방 장애물 차량과 중심거리를 이용해 좌측(양수)에 있는지 우측(음수)에 있는지 판단
//			curr_10_dist = this.getCarsDist(curr_toMiddle, change_toMiddle, curr_aicoars_10_middle, curr_track_curve_type, user_car_width_half, ai_car_length_half);
			
			// 차량 거리가 음수 이면 차량이 충돌하는 것으로 판단하여 우회
//			if(curr_01_dist <=  0 ){
//				System.out.println("+++++++++++++++++ 전방 장애물 발견 우회 하세요 ++++++++++++++++++++++");
//				if(curr_01_middle_dist > 0){ // 장애물 차가 우측에 존재(왼쪽으로 기본 이동)
//					user_change_dist[0] = change_toMiddle + ( curr_01_dist + user_car_width);
//				}else if(curr_01_middle_dist < 0){ // 장애물 차가 좌측에 존재(오른쪽으로 기본 이동)
//					user_change_dist[0] = change_toMiddle - ( curr_01_dist + user_car_width_half);
//				}else{
//					if(curr_track_curve_type == 1 ){ // 우회전 코스
//						user_change_dist[0] = change_toMiddle + ( curr_01_dist - user_car_width_half);
//					}else{ // 좌회전 코스
//						user_change_dist[0] = change_toMiddle - ( curr_01_dist + user_car_width_half);
//					}
//				}
//				
//				user_change_dist[1] = curr_aicoars_01_dist; // 전방 장애물차 거리
//			}else if(curr_01_dist > 0 && curr_01_dist < 0.5) {
//				System.out.println("+++++++++++++++++ 전방 장애물 발견 유지 하세요 ++++++++++++++++++++++");
//				if(curr_01_middle_dist > 0){ // 좌측에 존재
//					user_change_dist[0] = change_toMiddle + ( curr_aicoars_10_middle - curr_01_dist);
//				}else if(curr_01_middle_dist < 0){ // 우측에 존재
//					user_change_dist[0] = change_toMiddle - ( curr_aicoars_10_middle + curr_01_dist);
//				}else{
//					if(curr_track_curve_type == 1 ){ // 우회전 코스
//						user_change_dist[0] = change_toMiddle + ( curr_aicoars_10_middle - curr_01_dist);
//					}else{ // 좌회전 코스
//						user_change_dist[0] = change_toMiddle - ( curr_aicoars_10_middle + curr_01_dist);
//					}
//				}
//			}
			
			System.out.println("toMiddle         = "+change_toMiddle);
			System.out.println("user_change_dist = "+user_change_dist[0]);
			System.out.println("user_ai_car_dist = "+user_change_dist[1]);
			System.out.println("+++++++++++++++++ 장애물 처리 함수[end] ++++++++++++++++++++++");
		}

*/
//		System.out.println("change_toMiddle         = "+change_toMiddle);
//		System.out.println("user_change_dist        = "+user_change_dist[0]);
//		System.out.println("user_ai_car_dist        = "+user_change_dist[1]);
		
		return user_change_dist;
	}
	
	/**
	 * 차량간 실거리 계산
	 * @param user_car_toMiddle
	 * @param ai_car_toMiddle
	 * @param curr_track_curve_type
	 * @param user_car_width_half
	 * @param ai_car_length_half
	 * @return
	 */
	private double[] getCarsDist(double curr_toMiddle, double user_change_toMiddle, double ai_car_toMiddle, double curr_track_curve_type, double user_car_width, double ai_car_length){
		double[] cars_dist = new double[2];
		cars_dist[0] =  0; // 차량간 실거리
		cars_dist[1] =  user_change_toMiddle; // 사용자변경 toMiddle
		double cars_middle_dist = curr_toMiddle - ai_car_toMiddle; // 차량과의 중심 거리
		
		System.out.println("cars_middle_dist     = "+cars_middle_dist);
		// 차량간 실거리 (차폭을 고려하여 계산)
		cars_dist[0] = Math.abs(cars_middle_dist) - user_car_width/2 - ai_car_length/2;
		
		System.out.println("cars_dist            = "+cars_dist[0]);

		// 차량 거리가 음수 이면 차량이 충돌하는 것으로 판단하여 우회
		// TODO : 로직 변경 필요 차량 실거리
		if(cars_dist[0] <=  0 ){
			System.out.println("+++++++++++++++++ 전방 장애물 발견 우회 하세요 ++++++++++++++++++++++");
			if(cars_middle_dist > 0){ // 장애물 차가 우측에 존재(왼쪽으로 기본 이동)
				cars_dist[1] = user_change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
			}else if(cars_middle_dist < 0){ // 장애물 차가 좌측에 존재(오른쪽으로 기본 이동)
				cars_dist[1] = user_change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
			}else{
				if(curr_track_curve_type == 1 ){ // 우회전 코스(오른쪽으로 기본 이동)
					cars_dist[1] = user_change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
				}else{ // 좌회전 코스(왼쪽으로 기본 이동)
					cars_dist[1] = user_change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
				}
			}			
		}else if(cars_dist[0] > 0 && cars_dist[0] <= 0.5) {
			System.out.println("+++++++++++++++++ 전방 장애물 발견 유지 하세요 ++++++++++++++++++++++");
			if(cars_middle_dist > 0){ // 좌측에 존재
				cars_dist[1] = user_change_toMiddle + Math.abs(cars_dist[0]);
			}else if(cars_middle_dist < 0){ // 우측에 존재
				cars_dist[1] = user_change_toMiddle - Math.abs(cars_dist[0]);
			}else{
				if(curr_track_curve_type == 1 ){ // 우회전 코스
					cars_dist[1] = user_change_toMiddle + Math.abs(cars_dist[0]);
				}else{ // 좌회전 코스
					cars_dist[1] = user_change_toMiddle - Math.abs(cars_dist[0]);
				}
			}
		}else{
			return cars_dist;
		}
		return cars_dist;		
	}
	
	/**
	 * 브레이크, 엑셀 제어 함수
	 * @param curr_speed
	 * @param curr_best_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double[] getSpeedCtl(double curr_speed, double curr_best_speed, double curr_track_dist_straight){
		double[] user_speed_ctl = new double[2];
		user_speed_ctl[0] = 0.2; // accel
		user_speed_ctl[1] = 0.0; // brake

		if(curr_speed > curr_best_speed) {
			System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수[start] ++++++++++++++++++++++");
			System.out.println("curr_speed               = "+curr_speed);
			System.out.println("curr_best_speed          = "+curr_best_speed);
			System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			user_speed_ctl[0] = 0.0;
			
			if(curr_track_dist_straight < 20){
				user_speed_ctl[1] = 0.2;
			}else{
				user_speed_ctl[1] = 0.1;
			}
			System.out.println("user_brakeCtl="+user_speed_ctl[1]);
			System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수[end] ++++++++++++++++++++++");
			
		}else{
			user_speed_ctl[0] = 0.3;
		}		

		
		return user_speed_ctl;		
	}
	
	
	/**
	 *  최적 속도 계산
	 * @param curr_speed
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestSpeed(double curr_angle, double curr_dist_aicar, double curr_speed, double curr_track_dist_straight, double curr_track_curve_type){
		double user_best_speed = 100;
		
		float user_c_coeff      = (float)2.772;
		float user_d_coeff		= (float)-0.693;
		
		double curr_max_speed = 100;
		
		// 90 ~ 130 마일 일경우  초당 40.2~58.1m 이동 : 
		// 70 ~  90  마일 일경우 초당 31.2~40.2m 이동 : 
		//    ~  70  마일 일경우 초당 0   ~31.2m 이동 : 
				
		if(curr_track_dist_straight > 100){
			curr_max_speed = 80;
			
			user_c_coeff = (float)2.772;
			user_d_coeff = (float)-0.693;
		}else if (curr_track_dist_straight > 80 && curr_track_dist_straight <= 100) {
			curr_max_speed = 70;
			
			user_c_coeff = (float)2.0;
			user_d_coeff = (float)-0.9;
		}else if (curr_track_dist_straight > 50 && curr_track_dist_straight <= 80) {
			curr_max_speed = 50;
			
			user_c_coeff = (float)1.5;
			user_d_coeff = (float)-1.2;
		}else{
			curr_max_speed = 30; // 90도 이상일때 최적 속도(88~90km/h)
			
			user_c_coeff = (float)1.0;
			user_d_coeff = (float)-1.5;
		}
		
		double curr_angle_abs = Math.abs(curr_angle*180/3.14);
		if(curr_angle_abs <= 10){
			curr_max_speed = curr_max_speed*1.3;
		}else if(curr_angle_abs > 10 && curr_angle_abs <= 30 ){
			curr_max_speed = curr_max_speed*1;
		}else if(curr_angle_abs > 30 && curr_angle_abs <= 45 ){
			curr_max_speed = curr_max_speed*0.7;
		}else if(curr_angle_abs > 45 && curr_angle_abs <= 90 ){		
			curr_max_speed = curr_max_speed*0.6;
		}else if(curr_angle_abs > 90 && curr_angle_abs <= 135 ){
			curr_max_speed = curr_max_speed*0.5;
		}else if(curr_angle_abs > 135 && curr_angle_abs <= 180 ){
			curr_max_speed = curr_max_speed*0.2;
		}else{
			curr_max_speed = curr_max_speed*0.1;
		}
		 
//		/*테스트 셋팅 */
//		curr_max_speed = 20;
//		/*테스트 셋팅 */
		
		user_best_speed = curr_max_speed * (1 - Math.exp(-user_c_coeff/curr_max_speed * curr_dist_aicar - user_d_coeff));
		
		System.out.println("+++++++++++++++++ 최적 속도 계산[start] ++++++++++++++++++++++");
		System.out.println("user_c_coeff            ="+user_c_coeff);
		System.out.println("user_d_coeff            ="+user_d_coeff);
		System.out.println("curr_max_speed          ="+curr_max_speed);
		System.out.println("user_best_speed         ="+user_best_speed);
		System.out.println("curr_speed              ="+curr_speed + " m/s");
		System.out.println("curr_speed              ="+curr_speed*3.6 + " km/h");
		System.out.println("curr_angle              ="+curr_angle);
		System.out.println("curr_angle_abs          ="+curr_angle_abs + " 도");
		System.out.println("curr_track_dist_straight="+curr_track_dist_straight);
		System.out.println("+++++++++++++++++ 최적 속도 계산[end] ++++++++++++++++++++++");
		return user_best_speed;
	}
	
	
	
	/**
	 * 최적 이동 함수(기본)
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestDist(double curr_toMiddle, double curr_track_width, double curr_track_curve_type){
		double user_change_dist = 0.0;		
		
		if(curr_track_curve_type == 1){ // 우회전 
			user_change_dist = curr_toMiddle + ( curr_track_width/3 );  
		}else if(curr_track_curve_type == 2){ // 좌회전
			user_change_dist = curr_toMiddle - ( curr_track_width/3 );
		}else{
			user_change_dist = curr_toMiddle;
		}		
		
		return user_change_dist;		
	}	
	
	
	/**
	 * 핸들 제어 함수
	 * @param curr_angle
	 * @param change_dist
	 * @param curr_track_width
	 * @param streer_coeff
	 * @return
	 */
	private double getSteerCtl(double curr_angle, double change_dist, double curr_track_width, double streer_coeff) {
		double user_steerCtl = 0.0;
		
		user_steerCtl = streer_coeff * (curr_angle - change_dist/curr_track_width);
		
		return user_steerCtl;
	}
	
	/**
	 * 핸들 제어 계수 함수
	 * @param curr_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double getSteerCoeff(double curr_speed, double curr_track_dist_straight) {
		double steer_coeff = 0.5;
		
		if(curr_speed > 110 && curr_track_dist_straight > 20) {
			steer_coeff = 0.3;
		}
		return steer_coeff;
	}
	
	
	
	public static void main(String[] args) {
		DrivingInterface driving = new DrivingInterface();
		DrivingController2 controller = new DrivingController2();
		
		double[] driveArray = new double[DrivingInterface.INPUT_DRIVE_SIZE];
		double[] aicarArray = new double[DrivingInterface.INPUT_AICAR_SIZE];
		double[] trackArray = new double[DrivingInterface.INPUT_TRACK_SIZE];
		double[] damageArray = new double[DrivingInterface.INPUT_DAMAGE_SIZE];
		int[] rankArray = new int[DrivingInterface.INPUT_RANK_SIZE];
		int[] trackCurveType = new int[1];
		double[] trackAngleArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackDistArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackCurrentAngle = new double[1];
				
		// To-Do : Initialize with your team name.
		int result = driving.OpenSharedMemory();
		
		if(result == 0){
			boolean doLoop = true;
			while(doLoop){
				result = driving.ReadSharedMemory(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType, trackAngleArray, trackDistArray, trackCurrentAngle);
				switch(result){
				case 0:
					DrivingCmd cmd = controller.controlDriving(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType[0], trackAngleArray, trackDistArray, trackCurrentAngle[0]);
					driving.WriteSharedMemory(cmd.steer, cmd.accel, cmd.brake, cmd.backward);
					break;
				case 1:
					break;
				case 2:
					// disconnected
				default:
					// error occurred
					doLoop = false;
					break;
				}
			}
		}
	}
}

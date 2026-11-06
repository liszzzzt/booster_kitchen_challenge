"""内置 extension_process 扩展注册。"""

from core.extension_registry import register_extension, ParameterSpec, ExtensionProcessType


def register_builtin_extensions():
    """注册所有内置扩展

    所有扩展均使用 class_path 懒加载：注册时不触发任何 import，
    只有进程真正实例化扩展时才导入对应模块，彻底隔离各进程的依赖。
    """

    # DetectionExtension（extension_process）
    register_extension(
        name='detection_extension',
        class_path='extensions.detection_extension:DetectionExtension',
        parameters=[
            ParameterSpec(name='field_type', type=str, default='adult_size', required=False,
                          description="场地类型: 'adult_size' | 'kid_size' | 'custom'"),
            ParameterSpec(name='field_length',              type=float, default=0.0, required=False, description='场地长度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_width',               type=float, default=0.0, required=False, description='场地宽度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_penalty_dist',        type=float, default=0.0, required=False, description='罚球点距底线距离（custom 时必填，单位 m）'),
            ParameterSpec(name='field_goal_width',          type=float, default=0.0, required=False, description='球门宽度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_circle_radius',       type=float, default=0.0, required=False, description='中圈半径（custom 时必填，单位 m）'),
            ParameterSpec(name='field_penalty_area_length', type=float, default=0.0, required=False, description='罚球区长度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_penalty_area_width',  type=float, default=0.0, required=False, description='罚球区宽度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_goal_area_length',    type=float, default=0.0, required=False, description='球门区长度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_goal_area_width',     type=float, default=0.0, required=False, description='球门区宽度（custom 时必填，单位 m）'),
            ParameterSpec(name='field_goal_height',         type=float, default=1.8, required=False, description='球门高度，用于进球检测（单位 m）'),
            ParameterSpec(name='field_goal_depth',          type=float, default=0.6, required=False, description='球门深度，用于进球检测范围（单位 m）'),
            ParameterSpec(
                name='yolo_export_enabled',
                type=bool,
                default=False,
                required=False,
                description='是否启用YOLO数据集导出'
            ),
            ParameterSpec(
                name='yolo_export_dir',
                type=str,
                default='./yolo_dataset',
                required=False,
                description='YOLO数据集导出目录'
            ),
            ParameterSpec(
                name='yolo_export_skip_interval',
                type=int,
                default=6,
                required=False,
                description='每 N 帧导出一次 YOLO 数据'
            ),
            ParameterSpec(
                name='detection_fps',
                type=float,
                default=30.0,
                required=False,
                description='Detection 独立发布频率，上限 30Hz，不依赖 camera_frame FPS'
            ),
            ParameterSpec(
                name='worker_count',
                type=int,
                default=1,
                required=False,
                description='Detection 进程池 worker 数量；每个 worker 会加载一份 MuJoCo 模型'
            ),
            ParameterSpec(
                name='max_pending_tasks',
                type=int,
                default=2,
                required=False,
                description='Detection 待处理任务上限，超过后丢弃新帧以限制内存'
            ),
        ],
        description='Detection 扩展，运行在独立 ExtensionProcess 中，订阅 physics_state 定频执行 detection',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
        # Detection creates a private MuJoCo-backed worker pool inside the
        # extension, so it still needs the revisioned runtime scene even
        # though the outer ExtensionProcess worker does not preload a model.
        requires_runtime_scene=True,
    )

    # RgbPublisherExtension（extension_process）
    register_extension(
        name='rgb_publisher_extension',
        class_path='extensions.rgb_publisher_extension:RgbPublisherExtension',
        parameters=[
            ParameterSpec(
                name='enable_compression',
                type=bool,
                default=True,
                required=False,
                description='是否对 RGB 图像启用 JPEG 压缩'
            ),
            ParameterSpec(
                name='jpeg_quality',
                type=int,
                default=88,
                required=False,
                description='RGB JPEG 压缩质量 (1-100)'
            ),
        ],
        description='ROS2 RGB 图像发布扩展，运行在独立 ExtensionProcess 中，发布 RGB 图像和 rgb/camera_info',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=True,
    )

    register_extension(
        name='speaker_relay_extension',
        class_path='extensions.speaker_relay_extension:SpeakerRelayExtension',
        parameters=[
            ParameterSpec(name='topic', type=str, default='/speaker_audio', required=False,
                          description='ROS2 robot speaker AudioData topic'),
            ParameterSpec(name='publisher_url', type=str,
                          default='ws://127.0.0.1:9876/publish', required=False,
                          description='Media hub publisher ingress URL'),
        ],
        description='Relay robot Float32 PCM speaker audio from ROS2 to the browser',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
    )

    register_extension(
        name='pulse_speaker_bridge_extension',
        class_path='extensions.pulse_speaker_bridge_extension:PulseSpeakerBridgeExtension',
        parameters=[
            ParameterSpec(name='source', type=str,
                          default='alsa_output.usb-booster-virtual.monitor', required=False,
                          description='PulseAudio monitor source to capture playback from'),
            ParameterSpec(name='topic', type=str, default='/speaker_audio', required=False,
                          description='ROS2 robot speaker AudioData topic to publish to'),
            ParameterSpec(name='volume', type=float, default=1.0, required=False,
                          description='Output scale applied to captured PCM (0.0-1.0)'),
            ParameterSpec(name='drop_silence', type=bool, default=False, required=False,
                          description='Skip all-zero frames instead of publishing silence'),
        ],
        description='Capture robot audio service playback from PulseAudio and publish it as ROS2 speaker AudioData',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
    )

    register_extension(
        name='pulse_microphone_bridge_extension',
        class_path='extensions.pulse_microphone_bridge_extension:PulseMicrophoneBridgeExtension',
        parameters=[
            ParameterSpec(name='source', type=str,
                          default='alsa_input.usb-booster-virtual', required=False,
                          description='PulseAudio input source to expose to the SDK'),
            ParameterSpec(name='fifo', type=str, default='/tmp/booster_mic_fifo', required=False,
                          description='FIFO path used by module-pipe-source'),
            ParameterSpec(name='sample_rate', type=int, default=16000, required=False,
                          description='PulseAudio source rate in Hz'),
            ParameterSpec(name='volume', type=float, default=1.0, required=False,
                          description='Input scale applied to browser microphone PCM'),
        ],
        description='Capture browser microphone PCM from the camera hub and expose it as a PulseAudio input source for SDK recording',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=True,
    )

    # DepthPublisherExtension（extension_process）
    register_extension(
        name='depth_publisher_extension',
        class_path='extensions.depth_publisher_extension:DepthPublisherExtension',
        parameters=[],
        description='ROS2 深度图像发布扩展，运行在独立 ExtensionProcess 中，发布 depth/image_raw 和 depth/camera_info',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=True,
    )

    # E1R 3d-lidar
    register_extension(
        name="e1r_3d_lidar_control",
        class_path="extensions.e1r_3d_lidar_control:E1R3DLidarControl",
        parameters=[],
        description="Minimal E1R 3D LiDAR test",
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=True,
        requires_camera_client=False,
    )

    # PosePublisherExtension（extension_process）
    register_extension(
        name='pose_publisher_extension',
        class_path='extensions.pose_publisher_extension:PosePublisherExtension',
        parameters=[],
        description='ROS2 Pose2D 发布扩展，运行在独立 ExtensionProcess 中，自动检测所有 trunk body，发布 robot 的位置和方向',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
    )

    # ReplayStatePublisherExtension（extension_process）
    register_extension(
        name='replay_state_publisher',
        class_path='extensions.replay_state_publisher_extension:ReplayStatePublisherExtension',
        parameters=[
            ParameterSpec(name='target_hz', type=float, default=20.0, required=False,
                          description='3D replay 状态采样频率'),
            ParameterSpec(name='output_dir', type=str, default=None, required=False,
                          description='Replay Web artifact 输出目录；默认 MATCH_RESULT_DIR/replay'),
            ParameterSpec(name='chunk_size', type=int, default=1200, required=False,
                          description='每个 replay JSONL chunk 的最大帧数'),
        ],
        description='3D Replay Web artifact 写入扩展，输出机器人 root pose、关节角和球位置',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
    )

    # GameControlExtension（extension_process）
    register_extension(
        name='game_control',
        class_path='extensions.game_control.extension:GameControlExtension',
        parameters=[
            ParameterSpec(name='result_dir', type=str, default=None, required=False,
                          description='比赛结果目录；默认读取 MATCH_RESULT_DIR 或 logs/game-control'),
            ParameterSpec(name='runtime_dir', type=str, default=None, required=False,
                          description='比赛运行时目录；默认读取 MATCH_RUNTIME_DIR 或 result_dir/runtime'),
            ParameterSpec(name='http_host', type=str, default=None, required=False,
                          description='HTTP/SSE 状态接口监听地址；默认 GAME_CONTROL_HTTP_HOST 或 0.0.0.0'),
            ParameterSpec(name='http_port', type=int, default=None, required=False,
                          description='HTTP/SSE 状态接口端口；默认 GAME_CONTROL_HTTP_PORT 或 38383'),
            ParameterSpec(name='udp_address', type=str, default=None, required=False,
                          description='GameControl UDP 广播地址；默认 GAME_CONTROL_UDP_ADDRESS 或 255.255.255.255'),
            ParameterSpec(name='udp_port', type=int, default=None, required=False,
                          description='GameControl UDP 端口；默认 GAME_CONTROL_UDP_PORT 或 3838'),
            ParameterSpec(name='publish_hz', type=float, default=None, required=False,
                          description='GameControl 发布频率；默认 GAME_CONTROL_PUBLISH_HZ 或 2'),
            ParameterSpec(name='publish_transport', type=str, default=None, required=False,
                          description='GameControl 发布模式：udp 或 json_string；默认 GAME_CONTROL_PUBLISH_TRANSPORT 或 udp'),
            ParameterSpec(name='json_topic', type=str, default=None, required=False,
                          description='json_string 模式发布 std_msgs/String 的 ROS2 topic；默认 GAME_CONTROL_JSON_TOPIC 或 /soccer/game_controller'),
            ParameterSpec(name='players_per_team', type=int, default=None, required=False,
                          description='每队场上机器人数量；默认 GAME_CONTROL_PLAYERS_PER_TEAM 或 3'),
            ParameterSpec(name='home_team_number', type=int, default=None, required=False,
                          description='home 队 GameController team number；默认 GAME_CONTROL_HOME_TEAM_NUMBER 或 1'),
            ParameterSpec(name='away_team_number', type=int, default=None, required=False,
                          description='away 队 GameController team number；默认 GAME_CONTROL_AWAY_TEAM_NUMBER 或 2'),
            ParameterSpec(name='home_robots', type=str, default=None, required=False,
                          description='home 队 roster，格式 robotName:playerNumber[:role] 逗号分隔'),
            ParameterSpec(name='away_robots', type=str, default=None, required=False,
                          description='away 队 roster，格式 robotName:playerNumber[:role] 逗号分隔'),
            ParameterSpec(name='field_orientation', type=str, default=None, required=False,
                          description='场地方向：homeDefendsLeftGoal 或 homeDefendsRightGoal'),
            ParameterSpec(name='ground_truth_enabled', type=bool, default=None, required=False,
                          description='是否由 GameControl 发布 soccer ground truth topics'),
            ParameterSpec(name='ground_truth_ball_topic_template', type=str, default=None, required=False,
                          description='ground truth 足球 topic 模板，支持 {team_number}'),
            ParameterSpec(name='ground_truth_robot_pose_topic_template', type=str, default=None, required=False,
                          description='ground truth 机器人 pose topic 模板，支持 {team_number}/{sim_robot_name}'),
            ParameterSpec(name='ground_truth_robot_scope', type=str, default=None, required=False,
                          description='ground truth robot 发布范围：all_robots 或 own_team'),
            ParameterSpec(name='home_ground_truth_mirror', type=bool, default=None, required=False,
                          description='home 视角 ground truth 是否 180 度镜像'),
            ParameterSpec(name='away_ground_truth_mirror', type=bool, default=None, required=False,
                          description='away 视角 ground truth 是否 180 度镜像'),
            ParameterSpec(name='ready_timeout_ms', type=int, default=None, required=False,
                          description='等待队伍 ready 文件超时；默认 GAME_CONTROL_READY_TIMEOUT_MS 或 60000'),
            ParameterSpec(name='settle_ms', type=int, default=None, required=False,
                          description='ready 后开赛前稳定等待时间；默认 GAME_CONTROL_SETTLE_MS 或 2000'),
            ParameterSpec(name='require_team_ready', type=bool, default=None, required=False,
                          description='是否要求队伍 ready 文件；默认比赛模式开启，本地调试关闭'),
            ParameterSpec(name='exit_on_finish', type=bool, default=None, required=False,
                          description='比赛结束后 controller 是否停止；默认 headless 比赛开启'),
            ParameterSpec(name='auto_start', type=bool, default=None, required=False,
                          description='启动后是否自动进入自动裁判流程；默认比赛模式开启，本地调试关闭'),
            ParameterSpec(name='debug_publications', type=bool, default=None, required=False,
                          description='是否写入完整 GameControl publication 调试日志；默认关闭'),
            ParameterSpec(name='regular_duration_seconds', type=float, default=None, required=False,
                          description='常规比赛时间秒数；默认 GAME_CONTROL_REGULAR_DURATION_SECONDS 或 600'),
            ParameterSpec(name='draw_policy', type=str, default=None, required=False,
                          description='平局策略：allow_draw 或 extra_time；默认 GAME_CONTROL_DRAW_POLICY 或 allow_draw'),
            ParameterSpec(name='extra_time_max_seconds', type=float, default=None, required=False,
                          description='加时赛最长秒数；默认 GAME_CONTROL_EXTRA_TIME_MAX_SECONDS 或 12000'),
            ParameterSpec(name='initial_home_team_name', type=str, default=None, required=False,
                          description='恢复启动时 home 队名；默认 GAME_CONTROL_INITIAL_HOME_TEAM_NAME 或 home'),
            ParameterSpec(name='initial_away_team_name', type=str, default=None, required=False,
                          description='恢复启动时 away 队名；默认 GAME_CONTROL_INITIAL_AWAY_TEAM_NAME 或 away'),
            ParameterSpec(name='initial_home_external_team_id', type=str, default=None, required=False,
                          description='用户服务端 home 队伍 id；默认 GAME_CONTROL_INITIAL_HOME_EXTERNAL_TEAM_ID 或空'),
            ParameterSpec(name='initial_away_external_team_id', type=str, default=None, required=False,
                          description='用户服务端 away 队伍 id；默认 GAME_CONTROL_INITIAL_AWAY_EXTERNAL_TEAM_ID 或空'),
            ParameterSpec(name='initial_remaining_seconds', type=float, default=None, required=False,
                          description='恢复启动时主计时剩余秒数；默认 GAME_CONTROL_INITIAL_REMAINING_SECONDS 或 600'),
            ParameterSpec(name='initial_kicking_team_number', type=int, default=None, required=False,
                          description='恢复启动时开球队伍编号：1 或 2；默认 GAME_CONTROL_INITIAL_KICKING_TEAM_NUMBER 或 1'),
            ParameterSpec(name='initial_score_home', type=int, default=None, required=False,
                          description='恢复启动时 home 初始比分；默认 GAME_CONTROL_INITIAL_SCORE_HOME 或 0'),
            ParameterSpec(name='initial_score_away', type=int, default=None, required=False,
                          description='恢复启动时 away 初始比分；默认 GAME_CONTROL_INITIAL_SCORE_AWAY 或 0'),
        ],
        description='Python GameControl 扩展，作为权威裁判和 GameControl 发布者运行',
        process_type=ExtensionProcessType.EXTENSION_PROCESS,
        requires_mujoco_model=False,
        requires_camera_client=False,
    )

    print("[ExtensionRegistry] All builtin extensions registered")

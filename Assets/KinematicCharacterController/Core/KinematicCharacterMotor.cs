using System;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController
{
    public enum RigidbodyInteractionType
    {
        None,
        Kinematic,
        SimulatedDynamic
    }


    public enum StepHandlingMethod
    {
        None,
        Standard,
        Extra
    }


    /*
        在一个 update 中, 角色可能同时碰到数个 collider hit; 代码会逐个处理每个 hit对象;
        (所有 缓坡hit 都会被过滤掉, 它们不被本 state 考虑)
        -- state 会被先重置为 Initial, 直到遇到第一个 有效hit (陡坡/物体), 此时 state 变为 AfterFirstHit;
        -- 若再遇到一个 有效hit, 就要计算前后两个 hit 的关系, 以此确认角色是进入了 Crease 还是 Corner
    */
    public enum MovementSweepState
    {
        Initial,
        AfterFirstHit, // 若 hit对象是缓坡, 它会被过滤掉, 当state为 AfterFirstHit 时, 说明角色一定撞到了一个 陡坡/物体
        FoundBlockingCrease, // 发现 遮挡 褶皱:  猜测: 地面裂缝那种
        FoundBlockingCorner, // 发现 遮挡 角落:  猜测: 墙角那种
    }


    /// <summary>
    /// Represents the entire state of a character motor that is pertinent for simulation. -- 存储 Motor 全部信息
    /// Use this to save state or revert to past state
    /// </summary>
    [System.Serializable]
    public struct KinematicCharacterMotorState
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 BaseVelocity;

        public bool MustUnground;
        public float MustUngroundTime;
        public bool LastMovementIterationFoundAnyGround;
        public CharacterTransientGroundingReport GroundingStatus;

        public Rigidbody AttachedRigidbody;
        public Vector3 AttachedRigidbodyVelocity; // 可动平台提供的速度
    }

    /// <summary>
    /// Describes an overlap between the character capsule and another collider -- 
    /// 
    /// </summary>
    public struct OverlapResult
    {
        public Vector3 Normal;
        public Collider Collider;

        public OverlapResult(Vector3 normal, Collider collider)
        {
            Normal = normal;
            Collider = collider;
        }
    }

    /// <summary>
    /// Contains all the information for the motor's grounding status
    /// </summary>
    public struct CharacterGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround; // 当前是否稳定在 地面上, 如果斜坡太陡角色正在滑落, 或者角色跳到了空中, 此值都为 false
        public bool SnappingPrevented; // 防止 吸附 ?   猜测是主动把角色吸附在地面上
        public Vector3 GroundNormal;
        public Vector3 InnerGroundNormal;
        public Vector3 OuterGroundNormal;

        public Collider GroundCollider;
        public Vector3 GroundPoint;

        public void CopyFrom(CharacterTransientGroundingReport transientGroundingReport)
        {
            FoundAnyGround = transientGroundingReport.FoundAnyGround;
            IsStableOnGround = transientGroundingReport.IsStableOnGround;
            SnappingPrevented = transientGroundingReport.SnappingPrevented;
            GroundNormal = transientGroundingReport.GroundNormal;
            InnerGroundNormal = transientGroundingReport.InnerGroundNormal;
            OuterGroundNormal = transientGroundingReport.OuterGroundNormal;

            GroundCollider = null;
            GroundPoint = Vector3.zero;
        }
    }

    /// <summary>
    /// Contains the simulation-relevant information for the motor's grounding status
    /// </summary>
    public struct CharacterTransientGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround;
        public bool SnappingPrevented;
        public Vector3 GroundNormal;
        public Vector3 InnerGroundNormal;
        public Vector3 OuterGroundNormal;

        public void CopyFrom(CharacterGroundingReport groundingReport)
        {
            FoundAnyGround = groundingReport.FoundAnyGround;
            IsStableOnGround = groundingReport.IsStableOnGround;
            SnappingPrevented = groundingReport.SnappingPrevented;
            GroundNormal = groundingReport.GroundNormal;
            InnerGroundNormal = groundingReport.InnerGroundNormal;
            OuterGroundNormal = groundingReport.OuterGroundNormal;
        }
    }


    /// <summary>
    ///              Contains all the information from a hit stability evaluation -- 碰撞稳定性评估 
    ///              !! 爬楼梯的核心:
    ///              想象角色踩在一级 楼梯台阶 的边缘处; 胶囊体下半球某个点与台阶边缘相接; 这个点就是 hit点,               
    ///              从 hit点朝外(远离角色) 一小段距离 就是 out;  朝内一小段距离 就是 inn;
    /// 
    /// 
    /// 
    /// 
    /// </summary>
    public struct HitStabilityReport
    {
        // 如果碰到 陡坡, 此值为 false;
        // 若 _solveGrounding 设为 false, 此值将始终为 false
        public bool IsStable; 

        // 从 hitPoint 向内(靠近角色)微移一段距离, 然后再微抬高一点距离, 从此向下打射线(某段距离内), 查看是否能命中物体; 
        // 若能打中, 说明角色脚下有地面, 可以移动下来, 而不是掉落下来(暂猜)
        public bool FoundInnerNormal;  

        // 若 FoundInnerNormal == false, 本值为 innHitPos->角色下半球球心
        public Vector3 InnerNormal;

        // 从 hitPoint 向外(远离角色)微移一段距离, 然后再微抬高一点距离, 从此向下打射线(某段距离内), 查看是否能命中物体; 
        // 若能打中, 意味着 本次hit对象是个类似台阶的东西; 可以跨上去(暂猜)
        public bool FoundOuterNormal;

        // 若 FoundInnerNormal == false, 本值为 outHitPos->角色下半球球心
        public Vector3 OuterNormal;

        public bool ValidStepDetected;
        public Collider SteppedCollider;

        // ledge: 当 inn地面稳定性 和 out地面稳定性 不一致时, 此值为 true 
        // -- out 是缓坡, inn 是陡坡或空地
        // -- inn 是缓坡, out 是陡坡
        // 测试表明, 当 out 是高墙, 此值为 false ..
        public bool LedgeDetected;

        // 角色站在一个台阶边缘, 它out测的高台阶是个缓坡, inn测的低台阶不是缓坡; 
        // (看起来就像是 角色站在悬崖边, 且脚尖前是缓坡, 脚下却是空的 )
        // ---
        // 实践: 当角色踩在一个大台阶边缘, out侧是缓坡, inn侧是悬崖 or 陡坡, 此时本值为 true
        // 实践: 当角色踩在一个横圆木边缘, 此时本值为 false
        public bool IsOnEmptySideOfLedge;

        // hit点 到 角色 下半球球心的 向量, 投影在 角色up平面上, 得到的距离
        public float DistanceFromLedge;

        // 角色踩在台阶边缘上, inn为低阶, out为高台阶; 此时如果角色朝着 inn 方向走, 本值就为 true; 
        // 正如它的字面意思: 角色正走向 台阶边缘的 空区;
        public bool IsMovingTowardsEmptySideOfLedge;

        // 如果 out地面是缓坡, 就记录 out地面的法线, 否则记录 inn地面的法线;
        public Vector3 LedgeGroundNormal;

        // 鉴于 KCC 的 cross()错误尿性, 此变量很可能指向 left... 
        public Vector3 LedgeRightDirection;

        // 方向: 角色打向 hit点, (inn 打向 out 点)
        public Vector3 LedgeFacingDirection;
    }



    /// <summary>
    /// Contains the information of hit rigidbodies during the movement phase, so they can be processed afterwards
    /// </summary>
    public struct RigidbodyProjectionHit
    {
        public Rigidbody Rigidbody;
        public Vector3 HitPoint;
        public Vector3 EffectiveHitNormal;
        public Vector3 HitVelocity;
        public bool StableOnHit;
    }



    // ==============================================================================================================================


    /// <summary>
    /// Component that manages character collisions and movement solving
    /// </summary>
    [RequireComponent(typeof(CapsuleCollider))]
    public class KinematicCharacterMotor : MonoBehaviour
    {
#pragma warning disable 0414
        [Header("Components")]
        /// <summary>
        /// The capsule collider of this motor
        /// </summary>
        [ReadOnly]
        public CapsuleCollider Capsule;

        [Header("Capsule Settings")]
        /// <summary>
        /// Radius of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Radius of the Character Capsule")]
        private float CapsuleRadius = 0.5f;
        /// <summary>
        /// Height of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float CapsuleHeight = 2f;
        /// <summary>
        /// Local y position of the character's capsule center
        /// </summary>
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float CapsuleYOffset = 1f;
        /// <summary>
        /// Physics material of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Physics material of the Character Capsule (Does not affect character movement. Only affects things colliding with it)")]
#pragma warning disable 0649
        private PhysicMaterial CapsulePhysicsMaterial;
#pragma warning restore 0649


        [Header("Grounding settings")]
        /// <summary>
        /// Increases the range of ground detection, to allow snapping to ground at very high speeds -- 在运动速度很高时, 也能贴着地面运动, (而不是飞出去)
        /// </summary>    
        [Tooltip("Increases the range of ground detection, to allow snapping to ground at very high speeds")]
        public float GroundDetectionExtraDistance = 0f;

        /// <summary>
        /// Maximum slope angle on which the character can be stable  -- 最大斜坡角度,  超过后 角色将无法保持稳定 (这里的表现是滑下去)
        /// </summary>    
        [Range(0f, 89f)]
        [Tooltip("Maximum slope angle on which the character can be stable")]
        public float MaxStableSlopeAngle = 60f;

        /// <summary>
        /// Which layers can the character be considered stable on
        /// </summary>    
        [Tooltip("Which layers can the character be considered stable on")]
        public LayerMask StableGroundLayers = -1;
        /// <summary>
        /// Notifies the Character Controller when discrete collisions are detected  -- 检测到 离散 碰撞 时通知角色控制器
        /// </summary>    
        [Tooltip("Notifies the Character Controller when discrete collisions are detected")]
        public bool DiscreteCollisionEvents = false;


        [Header("Step settings")]
        /// <summary>
        /// Handles properly detecting grounding status on steps, but has a performance cost.
        /// </summary>
        [Tooltip("Handles properly detecting grounding status on steps, but has a performance cost.")]
        public StepHandlingMethod StepHandling = StepHandlingMethod.Standard;

        /// <summary>
        /// Maximum height of a step which the character can climb
        /// </summary>    
        [Tooltip("Maximum height of a step which the character can climb")]
        public float MaxStepHeight = 0.5f;

        /// <summary>
        /// Can the character step up obstacles even if it is not currently stable?  
        /// </summary>    
        [Tooltip("Can the character step up obstacles even if it is not currently stable?")]
        public bool AllowSteppingWithoutStableGrounding = false;
        /// <summary>
        /// Minimum length of a step that the character can step on (used in Extra stepping method. Use this to let the character step on steps that are smaller that its radius
        /// </summary>    
        [Tooltip("Minimum length of a step that the character can step on (used in Extra stepping method). Use this to let the character step on steps that are smaller that its radius")]
        public float MinRequiredStepDepth = 0.1f;


        [Header("Ledge settings")]
        /// <summary>
        /// Handles properly detecting ledge information and grounding status, but has a performance cost.  -- 是否处理 台阶边缘 status 和 着陆 status
        ///  拼错了, 应该是 "Denivellation": 不平度
        /// </summary>
        [Tooltip("Handles properly detecting ledge information and grounding status, but has a performance cost.")]
        public bool LedgeAndDenivelationHandling = true;

        /// <summary>
        /// The distance from the capsule central axis at which the character can stand on a ledge and still be stable
        /// </summary>    
        [Tooltip("The distance from the capsule central axis at which the character can stand on a ledge and still be stable")]
        public float MaxStableDistanceFromLedge = 0.5f;


        /// <summary>
        /// Prevents snapping to ground on ledges beyond a certain velocity 
        /// 此值变大后, 就算从 大尺寸台阶上下来, 也不会变成 "空中抛物线运动", 而是像下楼梯一样贴着地面下来
        /// </summary>    
        [Tooltip("Prevents snapping to ground on ledges beyond a certain velocity")]
        public float MaxVelocityForLedgeSnap = 0f;

        /// <summary>
        /// The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground  -- 角色可以承受并仍然捕捉到地面的最大向下倾斜角度变化
        /// </summary>    
        [Tooltip("The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground")]
        [Range(1f, 180f)]
        public float MaxStableDenivelationAngle = 180f;


        [Header("Rigidbody interaction settings")]
        /// <summary>
        /// Handles properly being pushed by and standing on PhysicsMovers or dynamic rigidbodies. Also handles pushing dynamic rigidbodies 
        /// 是否处理:
        ///     -1- 自己被 动态平台 或 其它动态 rigidbody 推动
        ///     -2- 自己推动 其它动态 rigidbody
        /// </summary>
        [Tooltip("Handles properly being pushed by and standing on PhysicsMovers or dynamic rigidbodies. Also handles pushing dynamic rigidbodies")]
        public bool InteractiveRigidbodyHandling = true;


        /// <summary>
        /// How the character interacts with non-kinematic rigidbodies.  
        ///  \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would).     -- 若对方是 运动刚体(isKinematic) 角色对它施加无限大的 推力
        ///  \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.                                           -- 若对方是 普通刚体              角色基于 mass 对他施加 推力
        /// </summary>
        [Tooltip("How the character interacts with non-kinematic rigidbodies. \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would). \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.")]
        public RigidbodyInteractionType RigidbodyInteractionType;


        [Tooltip("Mass used for pushing bodies")]
        public float SimulatedCharacterMass = 1f;


        /// <summary>
        /// Determines if the character preserves moving platform velocities when de-grounding from them  -- 确定角色在脱离平台时是否保持移动平台的速度
        /// </summary>
        [Tooltip("Determines if the character preserves moving platform velocities when de-grounding from them")]
        public bool PreserveAttachedRigidbodyMomentum = true;


        [Header("Constraints settings")]
        /// <summary>
        /// Determines if the character's movement uses the planar constraint 
        /// 施加 平面约束 后, 角色就只能在 法线为 PlanarConstraintAxis 的平面上运动;  猜测是为了满足某些特殊需求;
        /// </summary>
        [Tooltip("Determines if the character's movement uses the planar constraint")]
        public bool HasPlanarConstraint = false;

        /// <summary>
        /// Defines the plane that the character's movement is constrained on, if HasMovementConstraintPlane is active
        /// </summary>
        [Tooltip("Defines the plane that the character's movement is constrained on, if HasMovementConstraintPlane is active")]
        public Vector3 PlanarConstraintAxis = Vector3.forward;


        [Header("Other settings")]
        /// <summary>
        /// How many times can we sweep for movement per update -- 每次 update 我们可以 扫描 移动 多少次
        /// </summary>
        [Tooltip("How many times can we sweep for movement per update")]
        public int MaxMovementIterations = 5;
        /// <summary>
        /// How many times can we check for decollision per update
        /// </summary>
        [Tooltip("How many times can we check for decollision per update")]
        public int MaxDecollisionIterations = 1;

        /// <summary>
        /// Checks for overlaps before casting movement, making sure all collisions are detected even when already intersecting geometry (has a performance cost, but provides safety against tunneling through colliders)
        /// 在投射移动之前检查 是否存在 overlaps(撞到别的碰撞体)，确保即使已经相交的几何体也能检测到所有碰撞（具有性能成本，但可以安全地穿过碰撞器）
        /// </summary>
        [Tooltip("Checks for overlaps before casting movement, making sure all collisions are detected even when already intersecting geometry (has a performance cost, but provides safety against tunneling through colliders)")]
        public bool CheckMovementInitialOverlaps = true;

        /// <summary>
        /// Sets the velocity to zero if exceed max movement iterations -- 如果超过最大移动迭代次数，则将速度设置为零
        /// </summary>
        [Tooltip("Sets the velocity to zero if exceed max movement iterations")]
        public bool KillVelocityWhenExceedMaxMovementIterations = true;
        /// <summary>
        /// Sets the remaining movement to zero if exceed max movement iterations -- 如果超过最大移动迭代次数，则将剩余移动设置为零
        /// </summary>
        [Tooltip("Sets the remaining movement to zero if exceed max movement iterations")]
        public bool KillRemainingMovementWhenExceedMaxMovementIterations = true;

        /// <summary>
        /// Contains the current grounding information
        /// </summary>
        [System.NonSerialized]
        public CharacterGroundingReport GroundingStatus = new CharacterGroundingReport();

        /// <summary>
        /// Contains the previous grounding information
        /// </summary>
        [System.NonSerialized]
        public CharacterTransientGroundingReport LastGroundingStatus = new CharacterTransientGroundingReport();

        /// <summary>
        /// Specifies the LayerMask that the character's movement algorithm can detect collisions with. By default, this uses the rigidbody's layer's collision matrix
        /// </summary>
        [System.NonSerialized]
        public LayerMask CollidableLayers = -1;


        /// <summary>
        /// The Transform of the character motor -- 就是本 tf
        /// </summary>
        public Transform Transform { get { return _transform; } }
        private Transform _transform;


        /// <summary>
        /// The character's goal position in its movement calculations (always up-to-date (最新的) during the character update phase)
        ///  瞬态pos
        /// </summary>
        public Vector3 TransientPosition { get { return _transientPosition; } }
        private Vector3 _transientPosition;

        /// <summary>
        /// The character's up direction (always up-to-date (最新的) during the character update phase) 
        ///  貌似仅在 TransientRotation 中会被改写
        /// </summary>
        public Vector3 CharacterUp { get { return _characterUp; } }
        private Vector3 _characterUp;

        /// <summary>
        /// The character's forward direction (always up-to-date (最新的) during the character update phase)
        /// </summary>
        public Vector3 CharacterForward { get { return _characterForward; } }
        private Vector3 _characterForward;
        /// <summary>
        /// The character's right direction (always up-to-date (最新的) during the character update phase)
        /// </summary>
        public Vector3 CharacterRight { get { return _characterRight; } }
        private Vector3 _characterRight;


        
        /// <summary>
        /// The character's position before the movement calculations began
        /// </summary>
        public Vector3 InitialSimulationPosition { get { return _initialSimulationPosition; } }
        private Vector3 _initialSimulationPosition;
        /// <summary>
        /// The character's rotation before the movement calculations began
        /// </summary>
        public Quaternion InitialSimulationRotation { get { return _initialSimulationRotation; } }
        private Quaternion _initialSimulationRotation;


        /// <summary>
        /// Represents the Rigidbody to stay attached to 
        /// 猜测: 可动平台
        /// </summary>
        public Rigidbody AttachedRigidbody { get { return _attachedRigidbody; } }
        private Rigidbody _attachedRigidbody;

        
        /// <summary>
        /// Vector3 from the character transform position to the capsule center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleCenter { get { return _characterTransformToCapsuleCenter; } }
        private Vector3 _characterTransformToCapsuleCenter;

        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottom { get { return _characterTransformToCapsuleBottom; } }
        private Vector3 _characterTransformToCapsuleBottom;

        /// <summary>
        /// Vector3 from the character transform position to the capsule top
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTop { get { return _characterTransformToCapsuleTop; } }
        private Vector3 _characterTransformToCapsuleTop;


        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom hemi center 
        /// -- 从 capsule collider tf pos 到 胶囊体下半球球心 的 offset
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottomHemi { get { return _characterTransformToCapsuleBottomHemi; } }
        private Vector3 _characterTransformToCapsuleBottomHemi;


        /// <summary>
        /// Vector3 from the character transform position to the capsule top hemi center 
        /// -- 从 capsule collider tf pos 到 胶囊体上半球球心 的 offset
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTopHemi { get { return _characterTransformToCapsuleTopHemi; } }
        private Vector3 _characterTransformToCapsuleTopHemi;

        /// <summary>
        /// The character's velocity resulting from standing on rigidbodies or PhysicsMover
        /// -- 可动平台提供的速度
        /// </summary>
        public Vector3 AttachedRigidbodyVelocity { get { return _attachedRigidbodyVelocity; } }
        private Vector3 _attachedRigidbodyVelocity;

        /// <summary>
        /// The number of overlaps(重叠) detected so far during character update (is reset at the beginning of the update) 
        /// 在 character update 过程中, 累计被探查到的 overlaps 的数量 -- "overlap": 和本 capsule collider 重叠的另一个 collider;
        /// 本值在每个 update 开头被重置
        /// </summary>
        public int OverlapsCount { get { return _overlapsCount; } }
        private int _overlapsCount;

        /// <summary>
        /// The overlaps detected so far during character update -- "overlap": 和本 capsule collider 重叠的另一个 collider;
        /// </summary>
        public OverlapResult[] Overlaps { get { return _overlaps; } }
        private OverlapResult[] _overlaps = new OverlapResult[MaxRigidbodyOverlapsCount];

        /// <summary>
        /// The motor's assigned controller
        /// </summary>
        [NonSerialized]
        public ICharacterController CharacterController;


        /// <summary>
        /// Did the motor's last swept collision detection find a ground?
        /// </summary>
        [NonSerialized]
        public bool LastMovementIterationFoundAnyGround;


        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        [NonSerialized]
        public int IndexInCharacterSystem;


        /// <summary>
        /// Remembers initial position before all simulation are done; 每个循环开头时刻被缓存;
        /// 之后除非调用 SetPosition() SetPositionAndRotation(), 否则不会被改写
        /// </summary>
        [NonSerialized]
        public Vector3 InitialTickPosition;
        /// <summary>
        /// Remembers initial rotation before all simulation are done; 每个循环开头时刻被缓存;
        /// 之后除非调用 SetRotation() SetPositionAndRotation(), 否则不会被改写
        /// </summary>
        [NonSerialized]
        public Quaternion InitialTickRotation;


        /// <summary>
        /// Specifies a Rigidbody to stay attached to
        /// </summary>
        [NonSerialized]
        public Rigidbody AttachedRigidbodyOverride;

        /// <summary>
        /// The character's velocity resulting from direct movement
        /// </summary>
        [NonSerialized]
        public Vector3 BaseVelocity;

        // Private
        private RaycastHit[] _internalCharacterHits = new RaycastHit[MaxHitsBudget];
        private Collider[] _internalProbedColliders = new Collider[MaxCollisionBudget]; // 内部代码中 探测到的碰撞体, 有效元素会被紧致排列在前部
        private List<Rigidbody> _rigidbodiesPushedThisMove = new List<Rigidbody>(16);
        private RigidbodyProjectionHit[] _internalRigidbodyProjectionHits = new RigidbodyProjectionHit[MaxRigidbodyOverlapsCount];
        private Rigidbody _lastAttachedRigidbody;
        private bool _solveMovementCollisions = true; // whether or not the motor will solve collisions when moving (or moved onto)
        private bool _solveGrounding = true; // whether or not grounding will be evaluated for all hits -- 是否要检测每个 hit 的 着陆稳定性
        private bool _movePositionDirty = false;
        private Vector3 _movePositionTarget = Vector3.zero;
        private bool _moveRotationDirty = false;
        private Quaternion _moveRotationTarget = Quaternion.identity;
        private bool _lastSolvedOverlapNormalDirty = false;
        private Vector3 _lastSolvedOverlapNormal = Vector3.forward;
        private int _rigidbodyProjectionHitCount = 0;
        private bool _isMovingFromAttachedRigidbody = false;
        private bool _mustUnground = false;
        private float _mustUngroundTimeCounter = 0f;


        // 这组 cached 变量 只在此处被赋值:
        private Vector3 _cachedWorldUp = Vector3.up;
        private Vector3 _cachedWorldForward = Vector3.forward;
        private Vector3 _cachedWorldRight = Vector3.right;
        private Vector3 _cachedZeroVector = Vector3.zero;




        private Quaternion _transientRotation;
        /// <summary>
        /// The character's goal rotation in its movement calculations (always up-to-date (最新的) during the character update phase)
        /// 瞬态 rotation 
        /// 
        /// </summary>
        public Quaternion TransientRotation
        {
            get
            {
                return _transientRotation;
            }
            private set
            {
                _transientRotation = value;
                _characterUp = _transientRotation * _cachedWorldUp;
                _characterForward = _transientRotation * _cachedWorldForward;
                _characterRight = _transientRotation * _cachedWorldRight;
            }
        }


        /// <summary>
        /// The character's total velocity, including velocity from standing on rigidbodies or PhysicsMover
        /// </summary>
        public Vector3 Velocity
        {
            get
            {
                return this.BaseVelocity + this._attachedRigidbodyVelocity; // 叠加 可动平台 提供的速度
            }
        }

        // Warning: Don't touch these constants unless you know exactly what you're doing!
        public const int MaxHitsBudget = 16;
        public const int MaxCollisionBudget = 16;
        public const int MaxGroundingSweepIterations = 2;
        public const int MaxSteppingSweepIterations = 3;
        public const int MaxRigidbodyOverlapsCount = 16;
        public const float CollisionOffset = 0.01f; // 未避免真的碰到(相切) 而留的微小间隙
        public const float GroundProbeReboundDistance = 0.02f;
        public const float MinimumGroundProbingDistance = 0.005f;
        public const float GroundProbingBackstepDistance = 0.1f; //  执行 capsule cast 对地检测时, 会先将 capsule 向上移动一小步, 然后再向下 cast; 本值就是这个微小的后撤步距离
        public const float SweepProbingBackstepDistance = 0.002f; // 执行 capsule cast 检测时, 会先将 capsule 后撤一小步, 然后再超目标方向 cast; 本值就是这个微小的后撤步距离

        public const float SecondaryProbesVertical = 0.02f;
        public const float SecondaryProbesHorizontal = 0.001f; // origin:0.001f

        public const float MinVelocityMagnitude = 0.01f;
        public const float SteppingForwardDistance = 0.03f;
        public const float MinDistanceForLedge = 0.05f;
        public const float CorrelationForVerticalObstruction = 0.01f; // 垂直 障碍物 的相关性
        public const float ExtraSteppingForwardDistance = 0.01f;
        public const float ExtraStepHeightPadding = 0.01f;
#pragma warning restore 0414 

        private void OnEnable()
        {
            KinematicCharacterSystem.EnsureCreation();
            KinematicCharacterSystem.RegisterCharacterMotor(this);
        }

        private void OnDisable()
        {
            KinematicCharacterSystem.UnregisterCharacterMotor(this);
        }

        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        [ContextMenu("Remove Component")]
        private void HandleRemoveComponent()
        {
            CapsuleCollider tmpCapsule = gameObject.GetComponent<CapsuleCollider>();
            DestroyImmediate(this);
            DestroyImmediate(tmpCapsule);
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            Capsule = GetComponent<CapsuleCollider>();
            CapsuleRadius = Mathf.Clamp(CapsuleRadius, 0f, CapsuleHeight * 0.5f);
            Capsule.direction = 1;
            Capsule.sharedMaterial = CapsulePhysicsMaterial;
            SetCapsuleDimensions(CapsuleRadius, CapsuleHeight, CapsuleYOffset);

            MaxStepHeight = Mathf.Clamp(MaxStepHeight, 0f, Mathf.Infinity);
            MinRequiredStepDepth = Mathf.Clamp(MinRequiredStepDepth, 0f, CapsuleRadius);
            MaxStableDistanceFromLedge = Mathf.Clamp(MaxStableDistanceFromLedge, 0f, CapsuleRadius);

            transform.localScale = Vector3.one;

#if UNITY_EDITOR
            Capsule.hideFlags = HideFlags.NotEditable;
            if (!Mathf.Approximately(transform.lossyScale.x, 1f) || !Mathf.Approximately(transform.lossyScale.y, 1f) || !Mathf.Approximately(transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", this.gameObject);
            }
#endif
        }

        /// <summary>
        /// Sets whether or not the capsule collider will detect collisions
        /// </summary>
        public void SetCapsuleCollisionsActivation(bool collisionsActive)
        {
            Capsule.isTrigger = !collisionsActive;
        }

        /// <summary>
        /// Sets whether or not the motor will solve collisions when moving (or moved onto)
        /// </summary>
        public void SetMovementCollisionsSolvingActivation(bool movementCollisionsSolvingActive)
        {
            _solveMovementCollisions = movementCollisionsSolvingActive;
        }

        /// <summary>
        /// Sets whether or not grounding will be evaluated for all hits
        /// </summary>
        public void SetGroundSolvingActivation(bool stabilitySolvingActive)
        {
            _solveGrounding = stabilitySolvingActive;
        }

        /// <summary>
        /// Sets the character's position directly
        /// </summary>
        public void SetPosition(Vector3 position, bool bypassInterpolation = true)
        {
            _transform.position = position;
            this._initialSimulationPosition = position;
            this._transientPosition = position;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
            }
        }

        /// <summary>
        /// Sets the character's rotation directly
        /// </summary>
        public void SetRotation(Quaternion rotation, bool bypassInterpolation = true)
        {
            _transform.rotation = rotation;
            this._initialSimulationRotation = rotation;
            this.TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickRotation = rotation;
            }
        }

        /// <summary>
        /// Sets the character's position and rotation directly
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation, bool bypassInterpolation = true)
        {
            _transform.SetPositionAndRotation(position, rotation);
            this._initialSimulationPosition = position;
            this._initialSimulationRotation = rotation;
            this._transientPosition = position;
            this.TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
                InitialTickRotation = rotation;
            }
        }

        /// <summary>
        /// Moves the character position, taking all movement collision solving int account(暂记账上). The actual move is done the next time the motor updates are called
        /// </summary>
        public void MoveCharacter(Vector3 toPosition)
        {
            this._movePositionDirty = true;
            this._movePositionTarget = toPosition;
        }

        /// <summary>
        /// Moves the character rotation. The actual move is done the next time the motor updates are called -- 外部调用, 变化会延迟到下一帧被执行
        /// </summary>
        public void RotateCharacter(Quaternion toRotation)
        {
            _moveRotationDirty = true;
            _moveRotationTarget = toRotation;
        }

        /// <summary>
        /// Returns all the state information of the motor that is pertinent 直接相关的 for simulation
        ///  !!! 
        /// </summary>
        public KinematicCharacterMotorState GetState()
        {
            KinematicCharacterMotorState state = new KinematicCharacterMotorState();

            state.Position = this._transientPosition;
            state.Rotation = this._transientRotation;

            state.BaseVelocity = this.BaseVelocity;
            state.AttachedRigidbodyVelocity = this._attachedRigidbodyVelocity;

            state.MustUnground = _mustUnground;
            state.MustUngroundTime = _mustUngroundTimeCounter;
            state.LastMovementIterationFoundAnyGround = this.LastMovementIterationFoundAnyGround;
            state.GroundingStatus.CopyFrom(GroundingStatus);
            state.AttachedRigidbody = _attachedRigidbody;

            return state;
        }

        /// <summary>
        /// Applies a motor state instantly
        /// !!! 
        /// </summary>
        public void ApplyState(KinematicCharacterMotorState state, bool bypassInterpolation = true)
        {
            SetPositionAndRotation(state.Position, state.Rotation, bypassInterpolation);

            this.BaseVelocity = state.BaseVelocity;
            this._attachedRigidbodyVelocity = state.AttachedRigidbodyVelocity;

            _mustUnground = state.MustUnground;
            _mustUngroundTimeCounter = state.MustUngroundTime;
            this.LastMovementIterationFoundAnyGround = state.LastMovementIterationFoundAnyGround;
            GroundingStatus.CopyFrom(state.GroundingStatus);
            _attachedRigidbody = state.AttachedRigidbody;
        }

        /// <summary>
        /// Resizes capsule. ALso caches importand capsule size data
        /// </summary>
        public void SetCapsuleDimensions(float radius, float height, float yOffset)
        {
            height = Mathf.Max(height, (radius * 2f) + 0.01f); // Safety to prevent invalid capsule geometries

            CapsuleRadius = radius;
            CapsuleHeight = height;
            CapsuleYOffset = yOffset;

            Capsule.radius = CapsuleRadius;
            Capsule.height = Mathf.Clamp(CapsuleHeight, CapsuleRadius * 2f, CapsuleHeight);
            Capsule.center = new Vector3(0f, CapsuleYOffset, 0f);

            _characterTransformToCapsuleCenter = Capsule.center;
            _characterTransformToCapsuleBottom = Capsule.center + (-_cachedWorldUp * (Capsule.height * 0.5f));
            _characterTransformToCapsuleTop = Capsule.center + (_cachedWorldUp * (Capsule.height * 0.5f));
            _characterTransformToCapsuleBottomHemi = Capsule.center + (-_cachedWorldUp * (Capsule.height * 0.5f)) + (_cachedWorldUp * Capsule.radius);
            _characterTransformToCapsuleTopHemi = Capsule.center + (_cachedWorldUp * (Capsule.height * 0.5f)) + (-_cachedWorldUp * Capsule.radius);
        }

        private void Awake()
        {
            _transform = this.transform;
            ValidateData();

            this._transientPosition = _transform.position;
            this.TransientRotation = _transform.rotation;

            // Build CollidableLayers mask
            CollidableLayers = 0;
            for (int i = 0; i < 32; i++)
            {
                if (!Physics.GetIgnoreLayerCollision(this.gameObject.layer, i))
                {
                    CollidableLayers |= (1 << i);
                }
            }

            SetCapsuleDimensions(CapsuleRadius, CapsuleHeight, CapsuleYOffset);


            // ===================== tpr ==========================


            var Line_HitStablility_Inner    = VisualDebug.Instance.CreateLine( "Line_HitStablility_Inner", Color.red );
            var Line_HitStablility_Outer    = VisualDebug.Instance.CreateLine( "Line_HitStablility_Outer", Color.blue );
            var line_3tf                    = VisualDebug.Instance.CreateLine( "Line_3", Color.green );

            var Line_HitNormal_A            = VisualDebug.Instance.CreateLine( "Line_HitNormal_A", Color.yellow );

        }




        /// <summary>
        /// Update phase 1 is meant to be called after physics movers have calculated their velocities, but
        /// before they have simulated their goal positions/rotations. It is responsible for:
        /// - Initializing all values for update
        /// - Handling MovePosition calls
        /// - Solving initial collision overlaps
        /// - Ground probing  -- 地面探测
        /// - Handle detecting potential interactable rigidbodies
        /// </summary>
        public void UpdatePhase1(float deltaTime)
        {

            // NaN propagation safety stop
            // *** 注意, BaseVelocity 只有在出现异常时才会被重置; 否则会保留上一帧的值;
            if (float.IsNaN(this.BaseVelocity.x) || float.IsNaN(this.BaseVelocity.y) || float.IsNaN(this.BaseVelocity.z))
            {
                this.BaseVelocity = Vector3.zero;
            }

            // *** 同上, _attachedRigidbodyVelocity 在大部分时候也会保留上一帧的值;
            if (float.IsNaN(this._attachedRigidbodyVelocity.x) || float.IsNaN(this._attachedRigidbodyVelocity.y) || float.IsNaN(this._attachedRigidbodyVelocity.z))
            {
                this._attachedRigidbodyVelocity = Vector3.zero;
            }

#if UNITY_EDITOR
            if (!Mathf.Approximately(_transform.lossyScale.x, 1f) || !Mathf.Approximately(_transform.lossyScale.y, 1f) || !Mathf.Approximately(_transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", this.gameObject);
            }
#endif

            _rigidbodiesPushedThisMove.Clear();

            // Before update
            CharacterController.BeforeCharacterUpdate(deltaTime); // !!!-I

            this._transientPosition = _transform.position;
            this.TransientRotation = _transform.rotation;
            this._initialSimulationPosition = this._transientPosition;
            this._initialSimulationRotation = this._transientRotation;

            _rigidbodyProjectionHitCount = 0;
            _overlapsCount = 0;
            _lastSolvedOverlapNormalDirty = false;


            // 外部必须调用 MoveCharacter() 才能触发这段代码:
            #region Handle Move Position
            // todo: 此段暂时没走:
            if (this._movePositionDirty)
            {
                if (_solveMovementCollisions)
                {
                    Vector3 tmpVelocity = GetVelocityFromMovement(this._movePositionTarget - this._transientPosition, deltaTime); // 上一帧的 位移速度
                    if (InternalCharacterMove(ref tmpVelocity, deltaTime))
                    {
                        if (InteractiveRigidbodyHandling)
                        {
                            ProcessVelocityForRigidbodyHits(ref tmpVelocity, deltaTime);
                        }
                    }
                }
                else
                {
                    this._transientPosition = this._movePositionTarget;
                }

                this._movePositionDirty = false;
            }
            #endregion


            LastGroundingStatus.CopyFrom(GroundingStatus);
            GroundingStatus = new CharacterGroundingReport();
            GroundingStatus.GroundNormal = _characterUp;


            // 这段处理 所有和 角色capsule 相交的 colliders;
            // 针对每个 overlap, 角色会主动后退一段距离来解除相交; 并把每个 overlap 信息记录下来
            if (_solveMovementCollisions)
            {
                #region Resolve initial overlaps
                Vector3 resolutionDirection = _cachedWorldUp;
                float resolutionDistance = 0f;
                int iterationsMade = 0;

                bool overlapSolved = false;
                while (iterationsMade < MaxDecollisionIterations && !overlapSolved)
                {
                    int nbOverlaps = CharacterCollisionsOverlap(this._transientPosition, this._transientRotation, this._internalProbedColliders);

                    if (nbOverlaps > 0)
                    {
                        // Solve overlaps that aren't against dynamic rigidbodies or physics movers
                        for (int i = 0; i < nbOverlaps; i++)
                        {
                            if (GetInteractiveRigidbody(this._internalProbedColliders[i]) == null)
                            {
                                // 当这个 collider 要么不是 rigidbody, 要么是 运动刚体(代码控制) 
                                // 此时需要下方代码 手动解开 碰撞体相交(overlap): 

                                // Process overlap
                                Transform overlappedTransform = this._internalProbedColliders[i].GetComponent<Transform>();
                                // 判断两个 collider 是否相互内嵌, 如果是, 则可使用本函数来得知 相嵌信息, 进而可以将 它们 回退到相切的状态 (不内嵌);
                                if (Physics.ComputePenetration(
                                        // --- A:
                                        Capsule,
                                        this._transientPosition,
                                        this._transientRotation,
                                        // --- A:
                                        this._internalProbedColliders[i],
                                        overlappedTransform.position,
                                        overlappedTransform.rotation,
                                        // ---
                                        out resolutionDirection, // 方向: B->A
                                        out resolutionDistance))
                                {
                                    // Resolve along obstruction direction
                                    HitStabilityReport mockReport = new HitStabilityReport();
                                    mockReport.IsStable = IsStableOnNormal(resolutionDirection);
                                    resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport.IsStable);                                  

                                    // Solve overlap
                                    Vector3 resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                    this._transientPosition += resolutionMovement;

                                    // Remember overlaps
                                    if (_overlapsCount < _overlaps.Length)
                                    {
                                        _overlaps[_overlapsCount] = new OverlapResult(resolutionDirection, this._internalProbedColliders[i]);
                                        _overlapsCount++;
                                    }

                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        overlapSolved = true;
                    }

                    iterationsMade++;
                }
                #endregion
            }


            #region Ground Probing and Snapping 
            // Handle ungrounding
            if (_solveGrounding)
            {
                if (MustUnground())
                {
                    // --- 当前要求 角色"不接地", 比如准备跳跃时: ---

                    this._transientPosition += _characterUp * (MinimumGroundProbingDistance * 1.5f);
                }
                else
                {
                    // --- 当前要求 角色"接地": ---

                    // Choose the appropriate ground probing distance
                    float selectedGroundProbingDistance = MinimumGroundProbingDistance;
                    if (!LastGroundingStatus.SnappingPrevented && (LastGroundingStatus.IsStableOnGround || this.LastMovementIterationFoundAnyGround))  
                    {
                        if (StepHandling != StepHandlingMethod.None)
                        {
                            selectedGroundProbingDistance = Mathf.Max(CapsuleRadius, MaxStepHeight);
                        }
                        else
                        {
                            selectedGroundProbingDistance = CapsuleRadius;
                        }

                        selectedGroundProbingDistance += GroundDetectionExtraDistance;
                    }


                    ProbeGround(ref this._transientPosition, this._transientRotation, selectedGroundProbingDistance, ref GroundingStatus); 

                    // 上帧没找到地面, 本帧找到了
                    if (!LastGroundingStatus.IsStableOnGround && GroundingStatus.IsStableOnGround)
                    {
                        //print("BaseVelocity -1- = " + this.BaseVelocity.ToString() );
                        // 实测:  就算角色正在下落, (此行代码时) 它的 BaseVelocity.y 也依然为 0f; 有时又不是, 有点奇怪

                        // Handle stable landing
                        this.BaseVelocity = Vector3.ProjectOnPlane(this.BaseVelocity, CharacterUp);  
                        this.BaseVelocity = GetDirectionTangentToSurface(this.BaseVelocity, GroundingStatus.GroundNormal) * this.BaseVelocity.magnitude;
                    }
                }
            }


            this.LastMovementIterationFoundAnyGround = false;  // 感觉是重置状态

            if (_mustUngroundTimeCounter > 0f)
            {
                _mustUngroundTimeCounter -= deltaTime;
            }
            _mustUnground = false;
            #endregion


            if (_solveGrounding)
            {
                CharacterController.PostGroundingUpdate(deltaTime); // !!!-I
            }


            // !!! 晚点来看这段

            if (InteractiveRigidbodyHandling)
            {
                #region Interactive Rigidbody Handling 
                _lastAttachedRigidbody = _attachedRigidbody;
                if (AttachedRigidbodyOverride)
                {
                    _attachedRigidbody = AttachedRigidbodyOverride;
                }
                else
                {
                    // Detect interactive rigidbodies from grounding
                    if (GroundingStatus.IsStableOnGround && GroundingStatus.GroundCollider.attachedRigidbody)
                    {
                        Rigidbody interactiveRigidbody = GetInteractiveRigidbody(GroundingStatus.GroundCollider);
                        if (interactiveRigidbody)
                        {
                            _attachedRigidbody = interactiveRigidbody;
                        }
                    }
                    else
                    {
                        _attachedRigidbody = null;
                    }
                }

                Vector3 tmpVelocityFromCurrentAttachedRigidbody = Vector3.zero;
                Vector3 tmpAngularVelocityFromCurrentAttachedRigidbody = Vector3.zero;
                if (_attachedRigidbody)
                {
                    GetVelocityFromRigidbodyMovement(_attachedRigidbody, this._transientPosition, deltaTime, out tmpVelocityFromCurrentAttachedRigidbody, out tmpAngularVelocityFromCurrentAttachedRigidbody);
                }

                // Conserve momentum when de-stabilized from an attached rigidbody
                if (PreserveAttachedRigidbodyMomentum && _lastAttachedRigidbody != null && _attachedRigidbody != _lastAttachedRigidbody)
                {
                    this.BaseVelocity += this._attachedRigidbodyVelocity;
                    this.BaseVelocity -= tmpVelocityFromCurrentAttachedRigidbody;
                }

                // Process additionnal Velocity from attached rigidbody
                this._attachedRigidbodyVelocity = _cachedZeroVector;
                if (_attachedRigidbody)
                {
                    this._attachedRigidbodyVelocity = tmpVelocityFromCurrentAttachedRigidbody;

                    // Rotation from attached rigidbody
                    Vector3 newForward = Vector3.ProjectOnPlane(Quaternion.Euler(Mathf.Rad2Deg * tmpAngularVelocityFromCurrentAttachedRigidbody * deltaTime) * _characterForward, _characterUp).normalized;
                    this.TransientRotation = Quaternion.LookRotation(newForward, _characterUp);
                }

                // Cancel out horizontal velocity upon landing on an attached rigidbody
                if (GroundingStatus.GroundCollider &&
                    GroundingStatus.GroundCollider.attachedRigidbody &&
                    GroundingStatus.GroundCollider.attachedRigidbody == _attachedRigidbody &&
                    _attachedRigidbody != null &&
                    _lastAttachedRigidbody == null)
                {
                    this.BaseVelocity -= Vector3.ProjectOnPlane(this._attachedRigidbodyVelocity, _characterUp);
                }

                // Movement from Attached Rigidbody
                if (this._attachedRigidbodyVelocity.sqrMagnitude > 0f)
                {
                    _isMovingFromAttachedRigidbody = true;

                    if (_solveMovementCollisions)
                    {
                        // Perform the move from rgdbdy velocity
                        InternalCharacterMove(ref this._attachedRigidbodyVelocity, deltaTime);
                    }
                    else
                    {
                        this._transientPosition += this._attachedRigidbodyVelocity * deltaTime;
                    }

                    _isMovingFromAttachedRigidbody = false;
                }
                #endregion
            }
        }

        /// <summary>
        /// Update phase 2 is meant to be called after physics movers have simulated their goal positions/rotations. 
        /// At the end of this, the TransientPosition/Rotation values will be up-to-date (最新的) with where the motor should be at the end of its move. 
        /// It is responsible for:
        /// - Solving Rotation
        /// - Handle MoveRotation calls
        /// - Solving potential attached rigidbody overlaps
        /// - Solving Velocity
        /// - Applying planar constraint
        /// </summary>
        public void UpdatePhase2(float deltaTime)
        {
            // Handle rotation
            CharacterController.UpdateRotation(ref this._transientRotation, deltaTime); // !!!-I
            this.TransientRotation = this._transientRotation;

            // Handle move rotation
            if (_moveRotationDirty)
            {
                this.TransientRotation = _moveRotationTarget;
                _moveRotationDirty = false;
            }

            // !!! 晚点来看这段

            if (_solveMovementCollisions && InteractiveRigidbodyHandling)
            {
                if (InteractiveRigidbodyHandling)
                {
                    #region Solve potential attached rigidbody overlap
                    if (_attachedRigidbody)
                    {
                        float upwardsOffset = Capsule.radius;

                        RaycastHit closestHit;
                        if (CharacterGroundSweep(
                            this._transientPosition + (_characterUp * upwardsOffset),
                            this._transientRotation,
                            -_characterUp,
                            upwardsOffset,
                            out closestHit))
                        {
                            if (closestHit.collider.attachedRigidbody == _attachedRigidbody && IsStableOnNormal(closestHit.normal))
                            {
                                float distanceMovedUp = (upwardsOffset - closestHit.distance);
                                this._transientPosition = this._transientPosition + (_characterUp * distanceMovedUp) + (_characterUp * CollisionOffset);
                            }
                        }
                    }
                    #endregion
                }

                if (InteractiveRigidbodyHandling)
                {
                    #region Resolve overlaps that could've been caused by rotation or physics movers simulation pushing the character
                    Vector3 resolutionDirection = _cachedWorldUp;
                    float resolutionDistance = 0f;
                    int iterationsMade = 0;
                    bool overlapSolved = false;
                    while (iterationsMade < MaxDecollisionIterations && !overlapSolved)
                    {
                        int nbOverlaps = CharacterCollisionsOverlap(this._transientPosition, this._transientRotation, this._internalProbedColliders);
                        if (nbOverlaps > 0)
                        {
                            for (int i = 0; i < nbOverlaps; i++)
                            {
                                // Process overlap
                                Transform overlappedTransform = this._internalProbedColliders[i].GetComponent<Transform>();
                                if (Physics.ComputePenetration(
                                        Capsule,
                                        this._transientPosition,
                                        this._transientRotation,
                                        this._internalProbedColliders[i],
                                        overlappedTransform.position,
                                        overlappedTransform.rotation,
                                        out resolutionDirection,
                                        out resolutionDistance))
                                {
                                    // Resolve along obstruction direction
                                    HitStabilityReport mockReport = new HitStabilityReport();
                                    mockReport.IsStable = IsStableOnNormal(resolutionDirection);
                                    resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport.IsStable);

                                    // Solve overlap
                                    Vector3 resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                    this._transientPosition += resolutionMovement;

                                    // If interactiveRigidbody, register as rigidbody hit for velocity
                                    if (InteractiveRigidbodyHandling)
                                    {
                                        Rigidbody probedRigidbody = GetInteractiveRigidbody(this._internalProbedColliders[i]);
                                        if (probedRigidbody != null)
                                        {
                                            HitStabilityReport tmpReport = new HitStabilityReport();
                                            tmpReport.IsStable = IsStableOnNormal(resolutionDirection);
                                            if (tmpReport.IsStable)
                                            {
                                                this.LastMovementIterationFoundAnyGround = tmpReport.IsStable;
                                            }
                                            if (probedRigidbody != _attachedRigidbody)
                                            {
                                                Vector3 characterCenter = this._transientPosition + (this._transientRotation * _characterTransformToCapsuleCenter);
                                                Vector3 estimatedCollisionPoint = this._transientPosition;


                                                StoreRigidbodyHit(
                                                    probedRigidbody,
                                                    Velocity,
                                                    estimatedCollisionPoint,
                                                    resolutionDirection,
                                                    tmpReport);
                                            }
                                        }
                                    }

                                    // Remember overlaps
                                    if (_overlapsCount < _overlaps.Length)
                                    {
                                        _overlaps[_overlapsCount] = new OverlapResult(resolutionDirection, this._internalProbedColliders[i]);
                                        _overlapsCount++;
                                    }

                                    break;
                                }
                            }
                        }
                        else
                        {
                            overlapSolved = true;
                        }

                        iterationsMade++;
                    }
                    #endregion
                }
            }

            // Handle velocity
            CharacterController.UpdateVelocity(ref this.BaseVelocity, deltaTime); // !!!-I
            // 若当前在地面, 用户写的 BaseVelocity.y 为 0; 起跳后 y 为正值;  在空中自由落体时 y会受到 重力影响不断朝负无穷变化; 
            // 所以, 只要角色在地面 且 用户不输入, BaseVelocity 将保持为 0;
            // print( "UpdateVelocity -> BaseVelocity = " + BaseVelocity.ToString() );


            //this.CharacterController.UpdateVelocity(ref BaseVelocity, deltaTime);
            if (this.BaseVelocity.magnitude < MinVelocityMagnitude)
            {
                this.BaseVelocity = Vector3.zero;
            }

            #region Calculate Character movement from base velocity   
            // Perform the move from base velocity
            // 只要角色在地面 且 用户不输入, 下面这段就不会被执行到:
            if (this.BaseVelocity.sqrMagnitude > 0f)
            {
                if (_solveMovementCollisions)
                {
                    InternalCharacterMove(ref this.BaseVelocity, deltaTime);
                }
                else
                {
                    this._transientPosition += this.BaseVelocity * deltaTime;
                }
            }

            // Process rigidbody hits/overlaps to affect velocity
            if (InteractiveRigidbodyHandling)
            {
                ProcessVelocityForRigidbodyHits(ref this.BaseVelocity, deltaTime);
            }
            #endregion

            // Handle planar constraint 
            // 就是限制 角色只能在某个平面上运动, 冷门需求无需关注
            if (HasPlanarConstraint)
            {
                this._transientPosition = this._initialSimulationPosition + Vector3.ProjectOnPlane(this._transientPosition - this._initialSimulationPosition, PlanarConstraintAxis.normalized);
            }


            // Discrete collision detection -- "Discrete collision": 猜测是: 那些并不与角色capsule相交, 只是贴在一起的 colliders;
            if (DiscreteCollisionEvents)
            {
                // 注意最后一个参数, 角色 capsule 的 膨胀值; 这能检测出所有和 角色capsule 合法相切 的碰撞体, (比如 地面,墙面等)
                int nbOverlaps = CharacterCollisionsOverlap(this._transientPosition, this._transientRotation, this._internalProbedColliders, CollisionOffset * 2f);
                for (int i = 0; i < nbOverlaps; i++)
                {
                    CharacterController.OnDiscreteCollisionDetected(this._internalProbedColliders[i]); // !!!-I
                }
            }

            CharacterController.AfterCharacterUpdate(deltaTime); // !!!-I
        }


        /// <summary>
        /// Determines if motor can be considered stable on given slope normal
        /// </summary>
        private bool IsStableOnNormal(Vector3 normal)
        {
            return Vector3.Angle(_characterUp, normal) <= MaxStableSlopeAngle;
        }


        /// <summary>
        ///  Determines if motor can be considered stable on given slope normal 
        ///   
        /// </summary>
        private bool IsStableWithSpecialCases(ref HitStabilityReport stabilityReport, Vector3 velocity)
        {
            if (LedgeAndDenivelationHandling)
            {
                if (stabilityReport.LedgeDetected)
                {
                    if (stabilityReport.IsMovingTowardsEmptySideOfLedge)
                    {
                        // Max snap vel
                        // 当角色从 高台阶飞出时 (out->inn), 如果运动速度太快, 将不再贴地运动, 而是变成抛物线运动;
                        Vector3 velocityOnLedgeNormal = Vector3.Project(velocity, stabilityReport.LedgeFacingDirection);
                        if (velocityOnLedgeNormal.magnitude >= MaxVelocityForLedgeSnap)
                        {
                            return false;
                        }
                    }

                    // Distance from ledge
                    if (stabilityReport.IsOnEmptySideOfLedge && stabilityReport.DistanceFromLedge > MaxStableDistanceFromLedge)
                    {
                        return false;
                    }
                }

                // "Launching" off of slopes of a certain denivelation angle
                if (LastGroundingStatus.FoundAnyGround && stabilityReport.InnerNormal.sqrMagnitude != 0f && stabilityReport.OuterNormal.sqrMagnitude != 0f)
                {
                    float denivelationAngle = Vector3.Angle(stabilityReport.InnerNormal, stabilityReport.OuterNormal);
                    if (denivelationAngle > MaxStableDenivelationAngle)
                    {
                        return false;
                    }
                    else
                    {
                        denivelationAngle = Vector3.Angle(LastGroundingStatus.InnerGroundNormal, stabilityReport.OuterNormal);
                        if (denivelationAngle > MaxStableDenivelationAngle)
                        {
                            return false;
                        }
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Probes for valid ground and midifies the input transientPosition if ground snapping occurs 
        ///  整个过程会迭代多次, 不停地下探, 并把 参数 probingPosition 放到新的位置, 然后调整方向继续下探, 直到找到一个缓坡当作地面; 或者啥也没找到
        /// 
        /// 参数 groundingReport 会保留最后一次 hit 的相关信息;
        /// 
        /// </summary>
        public void ProbeGround(ref Vector3 probingPosition, Quaternion atRotation, float probingDistance, ref CharacterGroundingReport groundingReport) 
        {
            if (probingDistance < MinimumGroundProbingDistance)
            {
                probingDistance = MinimumGroundProbingDistance;
            }

            int groundSweepsMade = 0; // 扫描次数
            RaycastHit groundSweepHit = new RaycastHit();
            bool groundSweepingIsOver = false;
            Vector3 groundSweepPosition = probingPosition;
            Vector3 groundSweepDirection = (atRotation * -_cachedWorldUp); // 朝向 角色down 方向;
            float groundProbeDistanceRemaining = probingDistance;


            while ( groundProbeDistanceRemaining > 0 && (groundSweepsMade <= MaxGroundingSweepIterations) && !groundSweepingIsOver ) 
            {
                // Sweep for ground detection
                if (CharacterGroundSweep( 
                        groundSweepPosition, // position
                        atRotation, // rotation
                        groundSweepDirection, // direction
                        groundProbeDistanceRemaining, // distance
                        out groundSweepHit)) // hit -- 本次查出的 距离最近的 hit;
                {
                    // 若地面是斜的, 则 hit点不在 角色capsule 正下方, 而是下半球面某点上; 此时 角色下半球->hit点 是个斜的方向; (不是垂直向下)
                    // 但本变量只是让 角色垂直向下运动一段距离, 这么运动完后, 角色能与 本次斜坡 紧密相切; 但此时 targetPosition 并不是 hit点(groundSweepHit.point)
                    // 简单说, 只有当 地面完全水平时, targetPosition 和 groundSweepHit.point 才相同
                    Vector3 targetPosition = groundSweepPosition + (groundSweepDirection * groundSweepHit.distance);


                    HitStabilityReport groundHitStabilityReport = new HitStabilityReport();
                    EvaluateHitStability(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, targetPosition, this._transientRotation, this.BaseVelocity, ref groundHitStabilityReport, false);

                    groundingReport.FoundAnyGround = true;
                    groundingReport.GroundNormal = groundSweepHit.normal;
                    groundingReport.InnerGroundNormal = groundHitStabilityReport.InnerNormal;
                    groundingReport.OuterGroundNormal = groundHitStabilityReport.OuterNormal;
                    groundingReport.GroundCollider = groundSweepHit.collider;
                    groundingReport.GroundPoint = groundSweepHit.point;
                    groundingReport.SnappingPrevented = false;

                    // Found stable ground
                    if (groundHitStabilityReport.IsStable)
                    {
                        // Find all scenarios where ground snapping should be canceled
                        groundingReport.SnappingPrevented = !IsStableWithSpecialCases(ref groundHitStabilityReport, this.BaseVelocity);

                        groundingReport.IsStableOnGround = true;  // 说明 角色正稳稳地踩在地上

                        // *** "地面吸附" 执行处:
                        // Ground snapping
                        if (!groundingReport.SnappingPrevented)
                        {
                            probingPosition = groundSweepPosition + (groundSweepDirection * (groundSweepHit.distance - CollisionOffset));
                        }

                        // ** 执行到这里就意味着找到了一个有效的 地面, 也即将离开本函数了:
                        CharacterController.OnGroundHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, ref groundHitStabilityReport); // !!!-I
                        groundSweepingIsOver = true;
                    }
                    else
                    {
                        // ----- 说明本次hit 不是一个可以当作 地面的平面, 还需准备一下继续下一次迭代;

                        // Calculate movement from this iteration and advance position
                        Vector3 sweepMovement = (groundSweepDirection * groundSweepHit.distance) + ((atRotation * _cachedWorldUp) * Mathf.Max(CollisionOffset, groundSweepHit.distance));
                        groundSweepPosition = groundSweepPosition + sweepMovement;

                        // Set remaining distance
                        groundProbeDistanceRemaining = Mathf.Min(GroundProbeReboundDistance, Mathf.Max(groundProbeDistanceRemaining - sweepMovement.magnitude, 0f));

                        // Reorient direction
                        // 因为本次发现了 hit, 故原来的下探方向要被修正, 沿着 本次hit阻挡面的切方向继续下探
                        groundSweepDirection = Vector3.ProjectOnPlane(groundSweepDirection, groundSweepHit.normal).normalized;
                    }
                }
                else
                {
                    groundSweepingIsOver = true;
                }

                groundSweepsMade++;
            }
        }



        /// <summary>
        /// Forces the character to unground itself on its next grounding update
        ///  通常在起跳帧设置,  以便在之后的几帧 不主动执行 "snapped to the ground"; 不然角色会贴在地上跳不起来
        /// </summary>
        public void ForceUnground(float time = 0.1f)
        {
            _mustUnground = true;
            _mustUngroundTimeCounter = time;
        }

        // 是否为 "非接地" 状态, 能过滤掉 地面吸附功能
        public bool MustUnground()
        {
            return _mustUnground || _mustUngroundTimeCounter > 0f;
        }

        /// <summary>
        /// Returns the direction adjusted to be tangent to a specified surface normal relatively to the character's up direction.
        /// Useful for reorienting a direction on a slope without any lateral deviation(横向偏差 ; 横向偏移量) in trajectory(轨道)
        ///    得到 一个方向 在一个平面上的 切方向 (沿着这个平面运动的方向) 
        ///    得到的结果 归一化 了
        /// </summary>
        public Vector3 GetDirectionTangentToSurface(Vector3 direction, Vector3 surfaceNormal)
        {
            Vector3 directionRight = Vector3.Cross(direction, _characterUp); // 叉乘写反了
            return Vector3.Cross(surfaceNormal, directionRight).normalized; // 叉乘写反了
        }


        // !!!
        /// <summary>
        /// Moves the character's position by given movement while taking into account all physics simulation, step-handling and 
        /// velocity projection rules that affect the character motor
        /// </summary>
        /// <returns> Returns false if movement could not be solved until the end </returns>
        private bool InternalCharacterMove(ref Vector3 transientVelocity, float deltaTime)
        {
            if (deltaTime <= 0f)
                return false;

            bool wasCompleted = true;
            Vector3 remainingMovementDirection = transientVelocity.normalized;
            float remainingMovementMagnitude = transientVelocity.magnitude * deltaTime;
            Vector3 originalVelocityDirection = remainingMovementDirection;
            int sweepsMade = 0;     // 扫描次数
            bool hitSomethingThisSweepIteration = true; // 在这次扫描中是否 hit 某东西
            Vector3 tmpMovedPosition = this._transientPosition;
            bool previousHitIsStable = false;                   // (本帧内所有 hit对象 都会被遍历), 上一个遍历的 hit对象 是否为缓坡; 用来判断角色是否进入 Crease 或 Corner
            Vector3 previousVelocity = _cachedZeroVector;
            Vector3 previousObstructionNormal = _cachedZeroVector;
            MovementSweepState sweepState = MovementSweepState.Initial;

            // Project movement against current overlaps before doing the sweeps
            // "overlap": 和本 capsule collider 重叠的另一个 collider;
            //print("_overlapsCount = " + _overlapsCount);

            // !!! 在目前测试中,此处 _overlapsCount 始终为 0, 晚点来看...
            for (int i = 0; i < _overlapsCount; i++)
            {
                Vector3 overlapNormal = this._overlaps[i].Normal;

                // 当本次 hit对象 遮挡了原速度方向时: (否则这个 hit 会被过滤掉)
                if (Vector3.Dot(remainingMovementDirection, overlapNormal) < 0f)
                {
                    bool stableOnHit = IsStableOnNormal(overlapNormal) && !MustUnground(); // 坡度不能太大, 同时还要处于 "接地" 状态
                    Vector3 velocityBeforeProjection = transientVelocity;
                    Vector3 obstructionNormal = GetObstructionNormal(overlapNormal, stableOnHit);

                    InternalHandleVelocityProjection(
                        stableOnHit,
                        overlapNormal,
                        obstructionNormal,
                        originalVelocityDirection,
                        ref sweepState,
                        previousHitIsStable,
                        previousVelocity,
                        previousObstructionNormal,
                        ref transientVelocity,
                        ref remainingMovementMagnitude,
                        ref remainingMovementDirection);

                    previousHitIsStable = stableOnHit;
                    previousVelocity = velocityBeforeProjection;
                    previousObstructionNormal = obstructionNormal;
                }
            }

            // Sweep the desired movement to detect collisions
            while (remainingMovementMagnitude > 0f &&
                (sweepsMade <= MaxMovementIterations) &&
                hitSomethingThisSweepIteration)
            {
                // 下面这个 closest hit, 可能是当前正和 角色capsule 相交的, 也可能是位于 角色前进路线上的 距离最近的那个 hit
                bool foundClosestHit = false;               // 找到任何有效 collider 了
                Vector3 closestSweepHitPoint = default;     // 影响最大的collider 的 hit pos (在这个 collider 身上)
                Vector3 closestSweepHitNormal = default;    // 影响最大的collider 的 hit方向 (oth->self)
                float closestSweepHitDistance = 0f;         // 最接近的 扫描命中距离
                Collider closestSweepHitCollider = null;

                // 先从那些已经和 角色capsule 相嵌的 colliders 中查找:
                if (CheckMovementInitialOverlaps)
                {
                    int numOverlaps = CharacterCollisionsOverlap(
                                        tmpMovedPosition,
                                        this._transientRotation,
                                        this._internalProbedColliders,
                                        0f,         // 不膨胀
                                        false);

                    // 有效的 collider 元素个数
                    if (numOverlaps > 0)
                    {
                        closestSweepHitDistance = 0f;
                        float mostObstructingOverlapNormalDotProduct = 2f; // 只是用来找出 影响最大的 collider 用的

                        for (int i = 0; i < numOverlaps; i++)
                        {
                            Collider tmpCollider = this._internalProbedColliders[i];

                            // 判断两个 collider 是否相互内嵌, 如果是, 则可使用本函数来得知 相嵌信息, 进而可以将 它们 回退到相切的状态 (不内嵌);
                            if (Physics.ComputePenetration(
                                //--- A:
                                Capsule,
                                tmpMovedPosition,
                                this._transientRotation,
                                tmpCollider,
                                //--- B:
                                tmpCollider.transform.position,
                                tmpCollider.transform.rotation,
                                out Vector3 resolutionDirection,    //  oth->self
                                out float resolutionDistance))
                            {
                                float dotProduct = Vector3.Dot(remainingMovementDirection, resolutionDirection);
                                if (dotProduct < 0f && dotProduct < mostObstructingOverlapNormalDotProduct)
                                {
                                    mostObstructingOverlapNormalDotProduct = dotProduct;

                                    closestSweepHitNormal = resolutionDirection;
                                    closestSweepHitCollider = tmpCollider;
                                    closestSweepHitPoint = tmpMovedPosition + (this._transientRotation * CharacterTransformToCapsuleCenter) + (resolutionDirection * resolutionDistance);

                                    foundClosestHit = true;
                                }
                            }
                        }
                    }
                }

                // 如果上面的方法没找到 foundClosestHit,   说明当前 角色capsule 没和啥 collider 相交;
                // 于是改为检测 角色capsule 沿着 remainingMovementDirection 运动方向, 是否存在 hit; 找出距离最近的那个 hit 
                if (!foundClosestHit && CharacterCollisionsSweep(  // ** -- 不膨胀
                        tmpMovedPosition, // position
                        this._transientRotation, // rotation
                        remainingMovementDirection, // direction
                        remainingMovementMagnitude + CollisionOffset, // distance
                        out RaycastHit closestSweepHit, // closest hit
                        _internalCharacterHits) // all hits
                    > 0)
                {
                    closestSweepHitNormal = closestSweepHit.normal;
                    closestSweepHitDistance = closestSweepHit.distance;
                    closestSweepHitCollider = closestSweepHit.collider;
                    closestSweepHitPoint = closestSweepHit.point;

                    foundClosestHit = true;
                }


                if (foundClosestHit)
                {
                    // 只有当 角色capsule 在 move 过程中撞到什么东西时, foundClosestHit 才会为 true; 
                    // 比如 落地的第一帧, 撞墙的第一帧;  遇到台阶的头 一两帧(需再看下)

                    // ??? 地面坡度变化时, 是否会触发? 比如在崎岖地面上, 需测试...


                    // Calculate movement from this iteration
                    Vector3 sweepMovement = (remainingMovementDirection * (Mathf.Max(0f, closestSweepHitDistance - CollisionOffset))); // 角色合法的最长步进 offset
                    tmpMovedPosition += sweepMovement; // ** 直接让角色 前进到 紧贴着 本次遇到的 hit物;
                    remainingMovementMagnitude -= sweepMovement.magnitude;

                    // Evaluate if hit is stable
                    HitStabilityReport moveHitStabilityReport = new HitStabilityReport(); 
                    EvaluateHitStability(closestSweepHitCollider, closestSweepHitNormal, closestSweepHitPoint, tmpMovedPosition, this._transientRotation, transientVelocity, ref moveHitStabilityReport, true);

                    // todo: debug
                    // VisualDebug.Instance.DrawLine( 
                    //     "Line_HitStablility_Outer", 
                    //     closestSweepHitPoint,
                    //     closestSweepHitNormal,
                    //     2f,
                    //     0.08f 
                    // );

                    // Handle stepping up steps points higher than bottom capsule radius // !!! 处理高于 底部胶囊半径 的 步进点
                    bool foundValidStepHit = false;
                    if (_solveGrounding && StepHandling != StepHandlingMethod.None && moveHitStabilityReport.ValidStepDetected) 
                    {
                        float obstructionCorrelation = Mathf.Abs(Vector3.Dot(closestSweepHitNormal, _characterUp));
                        // 说明 hit法线 几乎和 _characterUp 垂直; 即: hit面是个接近垂直的面;
                        // 此时 hit点 一定高于 角色capsule 下半球球心; 就算 hit 是个楼梯台阶, 它的 step 也一定高于 角色capsule 半径;
                        if (obstructionCorrelation <= CorrelationForVerticalObstruction)
                        {

                            Vector3 stepForwardDirection = Vector3.ProjectOnPlane(-closestSweepHitNormal, _characterUp).normalized; // 水平向前的向量, self->hit

                            // 角色 脚底pos 往前移一点, 然后上移 MaxStepHeight; 如果 MaxStepHeight 足够大, 此时 角色capsule 和前面的 台阶collider 是不相交的;
                            Vector3 stepCastStartPoint = (tmpMovedPosition + (stepForwardDirection * SteppingForwardDistance)) + (_characterUp * MaxStepHeight);

                            // 然后向下做 capsule cast, 运动一段距离后, 角色capsule 下半球的某个点会撞到 台阶边缘直角处;

                            // Cast downward from the top of the stepping height
                            int nbStepHits = CharacterCollisionsSweep(
                                                stepCastStartPoint, // position
                                                this._transientRotation, // rotation
                                                -_characterUp, // direction -- 向下
                                                MaxStepHeight, // distance 
                                                out RaycastHit closestStepHit, // closest hit
                                                _internalCharacterHits,
                                                0f, // 角色 capsule 不膨胀
                                                true); // all hits 

                            // Check for hit corresponding to stepped collider
                            for (int i = 0; i < nbStepHits; i++)
                            {
                                if (_internalCharacterHits[i].collider == moveHitStabilityReport.SteppedCollider)
                                {
                                    // 
                                    Vector3 endStepPosition = stepCastStartPoint + (-_characterUp * (_internalCharacterHits[i].distance - CollisionOffset));

                                    // !!! 现在, 角色就会瞬移到 上面的 CharacterCollisionsSweep() 过程中触发碰撞的那个位置; 
                                    // 角色下半球的某个点撞到了 台阶边缘; // !!! 如果台阶很高, 且 MaxStepHeight 开的很大, 就能在游戏中发现角色在爬台阶时出现一帧瞬移, 角色被吸附到了新台阶上; 就是在此实现的;
                                    tmpMovedPosition = endStepPosition;
                                    foundValidStepHit = true;

                                    // Project velocity on ground normal at step
                                    transientVelocity = Vector3.ProjectOnPlane(transientVelocity, CharacterUp); // 砍掉 参数速度 的垂直分量, (因为找到并爬到台阶上了, 往后可以贴着新台阶(地面)运动了)
                                    remainingMovementDirection = transientVelocity.normalized;

                                    break;
                                }
                            }
                        }
                    }

                    // Handle movement solving
                    if (!foundValidStepHit) // !!! cur !!!
                    {
                        Vector3 obstructionNormal = GetObstructionNormal(closestSweepHitNormal, moveHitStabilityReport.IsStable);

                        // Movement hit callback
                        CharacterController.OnMovementHit(closestSweepHitCollider, closestSweepHitNormal, closestSweepHitPoint, ref moveHitStabilityReport); // !!!-I

                        // Handle remembering rigidbody hits
                        if (InteractiveRigidbodyHandling && closestSweepHitCollider.attachedRigidbody)
                        {
                            StoreRigidbodyHit(
                                closestSweepHitCollider.attachedRigidbody,
                                transientVelocity,
                                closestSweepHitPoint,
                                obstructionNormal,
                                moveHitStabilityReport);
                        }

                        bool stableOnHit = moveHitStabilityReport.IsStable && !MustUnground();
                        Vector3 velocityBeforeProj = transientVelocity;

                        // Project velocity for next iteration
                        InternalHandleVelocityProjection(
                            stableOnHit,
                            closestSweepHitNormal,
                            obstructionNormal,
                            originalVelocityDirection,
                            ref sweepState,
                            previousHitIsStable,
                            previousVelocity,
                            previousObstructionNormal,
                            ref transientVelocity,
                            ref remainingMovementMagnitude,
                            ref remainingMovementDirection);

                        previousHitIsStable = stableOnHit;
                        previousVelocity = velocityBeforeProj;
                        previousObstructionNormal = obstructionNormal;
                    }
                }
                // If we hit nothing...
                else
                {
                    hitSomethingThisSweepIteration = false;
                }

                // Safety for exceeding max sweeps allowed
                sweepsMade++;
                if (sweepsMade > MaxMovementIterations)
                {
                    if (KillRemainingMovementWhenExceedMaxMovementIterations)
                    {
                        remainingMovementMagnitude = 0f;
                    }

                    if (KillVelocityWhenExceedMaxMovementIterations)
                    {
                        transientVelocity = Vector3.zero;
                    }
                    wasCompleted = false;
                }
            }

            // Move position for the remainder of the movement
            tmpMovedPosition += (remainingMovementDirection * remainingMovementMagnitude);
            this._transientPosition = tmpMovedPosition;

            return wasCompleted;
        }


        /// <summary>
        /// Gets the effective normal for movement obstruction depending on current grounding status 
        /// 返回:
        ///     -- 要么 hit对象 是个缓坡, 返回这个缓坡的 法线
        ///     -- 要么 hit对象 是陡坡/物体, 会阻挡角色运动, 返回这个 hit对象 拒止角色运动的 平面的法线
        /// </summary>
        private Vector3 GetObstructionNormal(Vector3 hitNormal, bool stableOnHit) 
        {
            // Find hit/obstruction/offset normal
            Vector3 obstructionNormal = hitNormal;

            // 若: -1-角色站在稳定地面上, -2-角色处于"接地"状态, -3- hitNormal 不是缓坡
            // 这意味着 角色在地面上移动时撞到了一个 遮挡物, 需算出这个遮挡物 拒止角色运动的 平面的法线;
            if (GroundingStatus.IsStableOnGround && !MustUnground() && !stableOnHit)
            {
                Vector3 obstructionLeftAlongGround = Vector3.Cross(GroundingStatus.GroundNormal, obstructionNormal).normalized;
                obstructionNormal = Vector3.Cross(obstructionLeftAlongGround, _characterUp).normalized;
            }

            // Catch cases where cross product between parallel normals returned 0
            if (obstructionNormal.sqrMagnitude == 0f)
            {
                obstructionNormal = hitNormal;
            }

            return obstructionNormal;
        }


        /// <summary>
        /// Remembers a rigidbody hit for processing later
        /// </summary>
        private void StoreRigidbodyHit(Rigidbody hitRigidbody, Vector3 hitVelocity, Vector3 hitPoint, Vector3 obstructionNormal, HitStabilityReport hitStabilityReport)
        {
            if (_rigidbodyProjectionHitCount < _internalRigidbodyProjectionHits.Length)
            {
                if (!hitRigidbody.GetComponent<KinematicCharacterMotor>())
                {
                    RigidbodyProjectionHit rph = new RigidbodyProjectionHit();
                    rph.Rigidbody = hitRigidbody;
                    rph.HitPoint = hitPoint;
                    rph.EffectiveHitNormal = obstructionNormal;
                    rph.HitVelocity = hitVelocity;
                    rph.StableOnHit = hitStabilityReport.IsStable;

                    _internalRigidbodyProjectionHits[_rigidbodyProjectionHitCount] = rph;
                    _rigidbodyProjectionHitCount++;
                }
            }
        }

        public void SetTransientPosition(Vector3 newPos)
        {
            this._transientPosition = newPos;
        }



        /// <summary>
        /// Processes movement projection upon detecting a hit 
        /// 本函数是对 HandleVelocityProjection() 的进一步封装, 专门处理 一个 hit对象 对角色速度的影响;
        /// 
        /// </summary>
        private void InternalHandleVelocityProjection(bool stableOnHit, Vector3 hitNormal, Vector3 obstructionNormal, Vector3 originalDirection,
            ref MovementSweepState sweepState, bool previousHitIsStable, Vector3 previousVelocity, Vector3 previousObstructionNormal,
            ref Vector3 transientVelocity, ref float remainingMovementMagnitude, ref Vector3 remainingMovementDirection)
        {
            if (transientVelocity.sqrMagnitude <= 0f)
            {
                return;
            }

            Vector3 velocityBeforeProjection = transientVelocity;

            if (stableOnHit)
            {
                // --- hit对象 是个缓坡, 被当地面来处理:

                this.LastMovementIterationFoundAnyGround = true;
                HandleVelocityProjection(ref transientVelocity, obstructionNormal, stableOnHit);
            }
            else
            {
                // --- hit对象 是个陡坡/物体, 会阻挡本角色运动

                // Handle projection
                if (sweepState == MovementSweepState.Initial)
                {
                    HandleVelocityProjection(ref transientVelocity, obstructionNormal, stableOnHit);
                    sweepState = MovementSweepState.AfterFirstHit;
                }
                // Blocking crease handling
                else if (sweepState == MovementSweepState.AfterFirstHit)
                {
                    // --- 说明角色同时撞到了 第二个 有效hit;

                    EvaluateCrease(
                        transientVelocity,
                        previousVelocity,
                        obstructionNormal,
                        previousObstructionNormal,
                        stableOnHit,
                        previousHitIsStable,
                        GroundingStatus.IsStableOnGround && !MustUnground(),
                        out bool foundCrease,
                        out Vector3 creaseDirection);

                    if (foundCrease)
                    {
                        if (GroundingStatus.IsStableOnGround && !MustUnground())
                        {
                            //print("found Corner");
                            transientVelocity = Vector3.zero;
                            sweepState = MovementSweepState.FoundBlockingCorner;
                        }
                        else
                        {
                            //print("found Crease");
                            transientVelocity = Vector3.Project(transientVelocity, creaseDirection);
                            sweepState = MovementSweepState.FoundBlockingCrease;
                        }
                    }
                    else
                    {
                        HandleVelocityProjection(ref transientVelocity, obstructionNormal, stableOnHit);
                    }
                }
                // Blocking corner handling
                else if (sweepState == MovementSweepState.FoundBlockingCrease)
                {
                    transientVelocity = Vector3.zero;
                    sweepState = MovementSweepState.FoundBlockingCorner;
                }
            }

            float newVelocityFactor = transientVelocity.magnitude / velocityBeforeProjection.magnitude;
            remainingMovementMagnitude *= newVelocityFactor;
            remainingMovementDirection = transientVelocity.normalized;
        }



        // Crease 地缝
        private void EvaluateCrease(
            Vector3 currentCharacterVelocity,
            Vector3 previousCharacterVelocity,
            Vector3 currentHitNormal,   // 若 current hit 是陡坡/物体, 此平面将平行于 up方向, 否则就是个普通 hitNormal
            Vector3 previousHitNormal,  // 若 previous hit 是陡坡/物体, 此平面将平行于 up方向, 否则就是个普通 hitNormal
            bool currentHitIsStable,
            bool previousHitIsStable,
            bool characterIsStable,
            out bool isValidCrease,
            out Vector3 creaseDirection)
        {
            isValidCrease = false;
            creaseDirection = default;

            // -1-角色在空中;  -2-本次hit是 陡坡/物体;  -3- 上次hit是 陡坡/物体
            if (!characterIsStable || !currentHitIsStable || !previousHitIsStable)
            {
                Vector3 tmpBlockingCreaseDirection = Vector3.Cross(currentHitNormal, previousHitNormal).normalized; 
                float dotPlanes = Vector3.Dot(currentHitNormal, previousHitNormal);
                bool isVelocityConstrainedByCrease = false;

                // Avoid calculations if the two planes are the same
                if (dotPlanes < 0.999f)
                {
                    // TODO: can this whole part be made simpler? (with 2d projections, etc)
                    Vector3 normalAOnCreasePlane = Vector3.ProjectOnPlane(currentHitNormal, tmpBlockingCreaseDirection).normalized;
                    Vector3 normalBOnCreasePlane = Vector3.ProjectOnPlane(previousHitNormal, tmpBlockingCreaseDirection).normalized;
                    float dotPlanesOnCreasePlane = Vector3.Dot(normalAOnCreasePlane, normalBOnCreasePlane);

                    Vector3 enteringVelocityDirectionOnCreasePlane = Vector3.ProjectOnPlane(previousCharacterVelocity, tmpBlockingCreaseDirection).normalized;

                    if (dotPlanesOnCreasePlane <= (Vector3.Dot(-enteringVelocityDirectionOnCreasePlane, normalAOnCreasePlane) + 0.001f) &&
                        dotPlanesOnCreasePlane <= (Vector3.Dot(-enteringVelocityDirectionOnCreasePlane, normalBOnCreasePlane) + 0.001f))
                    {
                        isVelocityConstrainedByCrease = true;
                    }
                }

                if (isVelocityConstrainedByCrease)
                {
                    // Flip crease direction to make it representative of the real direction our velocity would be projected to
                    if (Vector3.Dot(tmpBlockingCreaseDirection, currentCharacterVelocity) < 0f)
                    {
                        tmpBlockingCreaseDirection = -tmpBlockingCreaseDirection;
                    }

                    isValidCrease = true;
                    creaseDirection = tmpBlockingCreaseDirection;
                }
            }
        }


        /// <summary>
        /// Allows you to override the way velocity is projected on an obstruction 
        /// 通用:  计算一个 hit对象(及其拒止平面) 对原速度的影响,  返回修正后的速度
        /// </summary>
        public virtual void HandleVelocityProjection(ref Vector3 velocity, Vector3 obstructionNormal, bool stableOnHit)
        {
            if (GroundingStatus.IsStableOnGround && !MustUnground())
            {
                // --- On stable slopes, simply reorient the movement without any loss
                //     角色在 稳定地面上运动, 且 hit对象是个缓坡, 计算返回 原速度在 hit缓坡上的 切线速度;
                if (stableOnHit)
                {
                    velocity = GetDirectionTangentToSurface(velocity, obstructionNormal) * velocity.magnitude; 
                }
                // --- On blocking hits, project the movement on the obstruction while following the grounding plane
                //     角色在 稳定地面上运动, 且 hit对象是个陡坡/物体, 计算返回 原速度被 hit阻挡后的 新速度; (它会沿着 拒止平面 和 GroundNormal平面 相交线 运动)
                else
                {
                    Vector3 obstructionRightAlongGround = Vector3.Cross(obstructionNormal, GroundingStatus.GroundNormal).normalized;
                    Vector3 obstructionUpAlongGround = Vector3.Cross(obstructionRightAlongGround, obstructionNormal).normalized; // 沿着拒止平面向上的方向, 感觉和 角色up相同
                    velocity = GetDirectionTangentToSurface(velocity, obstructionUpAlongGround) * velocity.magnitude; 
                    velocity = Vector3.ProjectOnPlane(velocity, obstructionNormal);
                }
            }
            else
            {
                // --- Handle stable landing
                //     角色在空中, 且 hit对象 是个缓坡, 可以着陆; 计算返回 原速度 在 hit缓坡 上的 切线速度;
                if (stableOnHit)
                {
                    velocity = Vector3.ProjectOnPlane(velocity, CharacterUp); // 去掉原速度的垂直分量
                    velocity = GetDirectionTangentToSurface(velocity, obstructionNormal) * velocity.magnitude;
                }
                // --- Handle generic obstruction
                //     角色在空中, 且 hit对象 是个陡坡/物体,  计算返回 原速度 被拒止平面挡住后 剩下的速度分量;
                else
                {
                    velocity = Vector3.ProjectOnPlane(velocity, obstructionNormal);
                }
            }
        }



        /// <summary>
        /// Allows you to override the way hit rigidbodies are pushed / interacted with. 
        /// ProcessedVelocity is what must be modified if this interaction affects the character's velocity.
        /// </summary>
        public virtual void HandleSimulatedRigidbodyInteraction(ref Vector3 processedVelocity, RigidbodyProjectionHit hit, float deltaTime)
        {
        }

        /// <summary>
        /// Takes into account rigidbody hits for adding to the velocity
        /// </summary>
        private void ProcessVelocityForRigidbodyHits(ref Vector3 processedVelocity, float deltaTime)
        {
            for (int i = 0; i < _rigidbodyProjectionHitCount; i++)
            {
                RigidbodyProjectionHit bodyHit = _internalRigidbodyProjectionHits[i];

                if (bodyHit.Rigidbody && !_rigidbodiesPushedThisMove.Contains(bodyHit.Rigidbody))
                {
                    if (_internalRigidbodyProjectionHits[i].Rigidbody != _attachedRigidbody)
                    {
                        // Remember we hit this rigidbody
                        _rigidbodiesPushedThisMove.Add(bodyHit.Rigidbody);

                        float characterMass = SimulatedCharacterMass;
                        Vector3 characterVelocity = bodyHit.HitVelocity;

                        KinematicCharacterMotor hitCharacterMotor = bodyHit.Rigidbody.GetComponent<KinematicCharacterMotor>();
                        bool hitBodyIsCharacter = hitCharacterMotor != null;
                        bool hitBodyIsDynamic = !bodyHit.Rigidbody.isKinematic;
                        float hitBodyMass = bodyHit.Rigidbody.mass;
                        float hitBodyMassAtPoint = bodyHit.Rigidbody.mass; // todo
                        Vector3 hitBodyVelocity = bodyHit.Rigidbody.velocity;
                        if (hitBodyIsCharacter)
                        {
                            hitBodyMass = hitCharacterMotor.SimulatedCharacterMass;
                            hitBodyMassAtPoint = hitCharacterMotor.SimulatedCharacterMass; // todo
                            hitBodyVelocity = hitCharacterMotor.BaseVelocity;
                        }
                        else if (!hitBodyIsDynamic)
                        {
                            PhysicsMover physicsMover = bodyHit.Rigidbody.GetComponent<PhysicsMover>();
                            if(physicsMover)
                            {
                                hitBodyVelocity = physicsMover.Velocity;
                            }
                        }

                        // Calculate the ratio of the total mass that the character mass represents
                        float characterToBodyMassRatio = 1f;
                        {
                            if (characterMass + hitBodyMassAtPoint > 0f)
                            {
                                characterToBodyMassRatio = characterMass / (characterMass + hitBodyMassAtPoint);
                            }
                            else
                            {
                                characterToBodyMassRatio = 0.5f;
                            }

                            // Hitting a non-dynamic body
                            if (!hitBodyIsDynamic)
                            {
                                characterToBodyMassRatio = 0f;
                            }
                            // Emulate kinematic body interaction
                            else if (RigidbodyInteractionType == RigidbodyInteractionType.Kinematic && !hitBodyIsCharacter)
                            {
                                characterToBodyMassRatio = 1f;
                            }
                        }

                        ComputeCollisionResolutionForHitBody(
                            bodyHit.EffectiveHitNormal,
                            characterVelocity,
                            hitBodyVelocity,
                            characterToBodyMassRatio,
                            out Vector3 velocityChangeOnCharacter,
                            out Vector3 velocityChangeOnBody);

                        processedVelocity += velocityChangeOnCharacter;

                        if (hitBodyIsCharacter)
                        {
                            hitCharacterMotor.BaseVelocity += velocityChangeOnCharacter;
                        }
                        else if (hitBodyIsDynamic)
                        {
                            bodyHit.Rigidbody.AddForceAtPosition(velocityChangeOnBody, bodyHit.HitPoint, ForceMode.VelocityChange);
                        }

                        if (RigidbodyInteractionType == RigidbodyInteractionType.SimulatedDynamic)
                        {
                            HandleSimulatedRigidbodyInteraction(ref processedVelocity, bodyHit, deltaTime);
                        }
                    }
                }
            }

        }

        public void ComputeCollisionResolutionForHitBody(
            Vector3 hitNormal,
            Vector3 characterVelocity,
            Vector3 bodyVelocity,
            float characterToBodyMassRatio,
            out Vector3 velocityChangeOnCharacter,
            out Vector3 velocityChangeOnBody)
        {
            velocityChangeOnCharacter = default;
            velocityChangeOnBody = default;

            float bodyToCharacterMassRatio = 1f - characterToBodyMassRatio;
            float characterVelocityMagnitudeOnHitNormal = Vector3.Dot(characterVelocity, hitNormal);
            float bodyVelocityMagnitudeOnHitNormal = Vector3.Dot(bodyVelocity, hitNormal);

            // if character velocity was going against the obstruction, restore the portion of the velocity that got projected during the movement phase
            if (characterVelocityMagnitudeOnHitNormal < 0f)
            {
                Vector3 restoredCharacterVelocity = hitNormal * characterVelocityMagnitudeOnHitNormal;
                velocityChangeOnCharacter += restoredCharacterVelocity;
            }

            // solve impulse velocities on both bodies, but only if the body velocity would be giving resistance to the character in any way
            if (bodyVelocityMagnitudeOnHitNormal > characterVelocityMagnitudeOnHitNormal)
            {
                Vector3 relativeImpactVelocity = hitNormal * (bodyVelocityMagnitudeOnHitNormal - characterVelocityMagnitudeOnHitNormal);
                velocityChangeOnCharacter += relativeImpactVelocity * bodyToCharacterMassRatio;
                velocityChangeOnBody += -relativeImpactVelocity * characterToBodyMassRatio;
            }
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        /// <returns> Returns true if the collider is valid </returns>
        private bool CheckIfColliderValidForCollisions(Collider coll) // see__
        {
            // Ignore self
            if (coll == Capsule)
            {
                return false;
            }

            if (!InternalIsColliderValidForCollisions(coll))
            {
                return false;
            }

            return true;
        }


        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        private bool InternalIsColliderValidForCollisions(Collider coll)
        {
            Rigidbody colliderAttachedRigidbody = coll.attachedRigidbody;
            if (colliderAttachedRigidbody)
            {
                bool isRigidbodyKinematic = colliderAttachedRigidbody.isKinematic; // true: 对方是运动刚体, 不受引擎控制, 受代码控制

                // If movement is made from AttachedRigidbody, ignore the AttachedRigidbody
                if (_isMovingFromAttachedRigidbody && (!isRigidbodyKinematic || colliderAttachedRigidbody == _attachedRigidbody))
                {
                    return false;
                }

                // don't collide with dynamic rigidbodies if our RigidbodyInteractionType is kinematic
                // 本角色启用 Kinematic 模式, 且对方是普通刚体
                if (RigidbodyInteractionType == RigidbodyInteractionType.Kinematic && !isRigidbodyKinematic)
                {
                    // wake up rigidbody
                    if (coll.attachedRigidbody)
                    {
                        coll.attachedRigidbody.WakeUp();
                    }

                    return false;
                }
            }

            // Custom checks
            bool colliderValid = CharacterController.IsColliderValidForCollisions(coll); // !!!-I  用户来决定这个 collider 是否合法
            if (!colliderValid)
            {
                return false;
            }

            return true;
        }


        /// <summary>
        /// Determines if the motor is considered stable on a given hit
        /// </summary>
        public void EvaluateHitStability(   // !!! cur  复读
            Collider hitCollider, 
            Vector3 hitNormal, 
            Vector3 hitPoint,               // hit点, 如果是高于角色半径的墙, 此点就在 角色capsule下半球球心上方 任意位置(不固定); // !!! cur
            Vector3 atCharacterPosition,    // 角色capsule 底部点 pos; 只有当 hitCollider 是个水平地面时, 此值才和 hitPoint 相同
            Quaternion atCharacterRotation, 
            Vector3 withCharacterVelocity, 
            ref HitStabilityReport stabilityReport,
            bool isNeedDebug = false // 过滤掉 地面探测那次调用
        ){

            if (!_solveGrounding)
            {
                stabilityReport.IsStable = false;
                return;
            }

            Vector3 atCharacterUp = atCharacterRotation * _cachedWorldUp; // 角色的 up
            Vector3 innerHitDirection = Vector3.ProjectOnPlane(hitNormal, atCharacterUp).normalized; // hit方向(hit->self) 在角色xz平面上的 投影方向

            stabilityReport.IsStable = this.IsStableOnNormal(hitNormal);

            stabilityReport.FoundInnerNormal = false;
            stabilityReport.FoundOuterNormal = false;
            stabilityReport.InnerNormal = hitNormal;
            stabilityReport.OuterNormal = hitNormal;

            // Ledge handling
            if (LedgeAndDenivelationHandling)
            {
                // 探测距离
                float ledgeCheckHeight = MinDistanceForLedge;
                if (StepHandling != StepHandlingMethod.None)
                {
                    ledgeCheckHeight = MaxStepHeight;
                }

                bool isStableLedgeInner = false; // 若检测到 inn地面, inn地面 是否为缓坡 
                bool isStableLedgeOuter = false; // 若检测到 out地面, out地面 是否为缓坡 

                if (CharacterCollisionsRaycast(
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (innerHitDirection * SecondaryProbesHorizontal),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        out RaycastHit innerLedgeHit,
                        _internalCharacterHits) > 0)
                {
                    // todo: debug:
                    if(isNeedDebug)
                    {
                        TextDebug.SetText( 1, "inn - true" );
                        // VisualDebug.Instance.DrawLine( 
                        //     "Line_HitStablility_Inner", 
                        //     hitPoint + (atCharacterUp * SecondaryProbesVertical) + (innerHitDirection * 0.08f),
                        //     -atCharacterUp,
                        //     ledgeCheckHeight + SecondaryProbesVertical,
                        //     0.05f 
                        // );
                    }
                    

                    Vector3 innerLedgeNormal = innerLedgeHit.normal;
                    stabilityReport.InnerNormal = innerLedgeNormal;
                    stabilityReport.FoundInnerNormal = true;
                    isStableLedgeInner = IsStableOnNormal(innerLedgeNormal);
                    if(isNeedDebug)
                    {
                        print("inn = " + isStableLedgeInner );
                    }
                }
                else 
                {
                    // todo: debug:
                    if(isNeedDebug)
                    {
                        TextDebug.SetText( 1, "---" );
                        print("inn = false " );
                    }
                }

                if(isNeedDebug)
                {
                    VisualDebug.Instance.DrawLine( 
                        "Line_HitStablility_Inner", 
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (innerHitDirection * 0.08f),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        0.05f 
                    );
                }

                if (CharacterCollisionsRaycast(
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        out RaycastHit outerLedgeHit,
                        _internalCharacterHits) > 0)
                {
                    // todo: debug:
                    if(isNeedDebug)
                    {
                        TextDebug.SetText( 2, "out - true" );
                        // VisualDebug.Instance.DrawLine( 
                        //     "Line_HitStablility_Outer", 
                        //     hitPoint + (atCharacterUp * SecondaryProbesVertical) + (-innerHitDirection * 0.08f),
                        //     -atCharacterUp,
                        //     ledgeCheckHeight + SecondaryProbesVertical,
                        //     0.05f 
                        // );
                    }

                    Vector3 outerLedgeNormal = outerLedgeHit.normal;
                    stabilityReport.OuterNormal = outerLedgeNormal;
                    stabilityReport.FoundOuterNormal = true;
                    isStableLedgeOuter = IsStableOnNormal(outerLedgeNormal);
                    if(isNeedDebug)
                    {
                        print(" out = " + isStableLedgeOuter );
                    }
                }
                else 
                {
                    // todo: debug:
                    if(isNeedDebug)
                    {
                        TextDebug.SetText( 2, "---" );
                    }
                }

                if(isNeedDebug)
                {
                    // todo: debug:
                    VisualDebug.Instance.DrawLine( 
                        "Line_HitStablility_Outer", 
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (-innerHitDirection * 0.08f),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        0.05f 
                    );
                }
                

                stabilityReport.LedgeDetected = (isStableLedgeInner != isStableLedgeOuter);

                if(isNeedDebug)
                {
                    TextDebug.SetText( 3, "LedgeDetected = " +  stabilityReport.LedgeDetected);
                }

                if (stabilityReport.LedgeDetected)
                {
                    if(isNeedDebug)
                    {
                        //print("inn = " + isStableLedgeInner + ", out = " + isStableLedgeOuter );
                    }
                    stabilityReport.IsOnEmptySideOfLedge = isStableLedgeOuter && !isStableLedgeInner;
                    stabilityReport.LedgeGroundNormal = isStableLedgeOuter ? stabilityReport.OuterNormal : stabilityReport.InnerNormal;
                    stabilityReport.LedgeRightDirection = Vector3.Cross(hitNormal, stabilityReport.LedgeGroundNormal).normalized;
                    stabilityReport.LedgeFacingDirection = Vector3.ProjectOnPlane(Vector3.Cross(stabilityReport.LedgeGroundNormal, stabilityReport.LedgeRightDirection), CharacterUp).normalized;
                    stabilityReport.DistanceFromLedge = Vector3.ProjectOnPlane((hitPoint - (atCharacterPosition + (atCharacterRotation * _characterTransformToCapsuleBottom))), atCharacterUp).magnitude;
                    stabilityReport.IsMovingTowardsEmptySideOfLedge = Vector3.Dot(withCharacterVelocity.normalized, stabilityReport.LedgeFacingDirection) > 0f;
                }

                if (stabilityReport.IsStable)
                {
                    stabilityReport.IsStable = IsStableWithSpecialCases(ref stabilityReport, withCharacterVelocity);
                }
            }

            // Step handling
            if (StepHandling != StepHandlingMethod.None && !stabilityReport.IsStable)
            {
                // Stepping not supported on dynamic rigidbodies
                Rigidbody hitRigidbody = hitCollider.attachedRigidbody;
                if (!(hitRigidbody && !hitRigidbody.isKinematic))
                {
                    DetectSteps(atCharacterPosition, atCharacterRotation, hitPoint, innerHitDirection, ref stabilityReport);

                    if (stabilityReport.ValidStepDetected)
                    {
                        stabilityReport.IsStable = true;
                    }
                }
            }

            CharacterController.ProcessHitStabilityReport(hitCollider, hitNormal, hitPoint, atCharacterPosition, atCharacterRotation, ref stabilityReport); // !!!-I
        }

        private void DetectSteps(Vector3 characterPosition, Quaternion characterRotation, Vector3 hitPoint, Vector3 innerHitDirection, ref HitStabilityReport stabilityReport)
        {
            int nbStepHits = 0;
            Collider tmpCollider;
            RaycastHit outerStepHit;
            Vector3 characterUp = characterRotation * _cachedWorldUp;
            Vector3 verticalCharToHit = Vector3.Project((hitPoint - characterPosition), characterUp);
            Vector3 horizontalCharToHitDirection = Vector3.ProjectOnPlane((hitPoint - characterPosition), characterUp).normalized;
            Vector3 stepCheckStartPos = (hitPoint - verticalCharToHit) + (characterUp * MaxStepHeight) + (horizontalCharToHitDirection * CollisionOffset * 3f); 

            // Do outer step check with capsule cast on hit point
            nbStepHits = CharacterCollisionsSweep(
                            stepCheckStartPos,
                            characterRotation,
                            -characterUp,
                            MaxStepHeight + CollisionOffset,
                            out outerStepHit,
                            _internalCharacterHits,
                            0f,
                            true);

            // Check for overlaps and obstructions at the hit position
            if (CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out tmpCollider))
            {
                stabilityReport.ValidStepDetected = true;
                stabilityReport.SteppedCollider = tmpCollider;
            }

            if (StepHandling == StepHandlingMethod.Extra && !stabilityReport.ValidStepDetected)
            {
                // Do min reach step check with capsule cast on hit point
                stepCheckStartPos = characterPosition + (characterUp * MaxStepHeight) + (-innerHitDirection * MinRequiredStepDepth);
                nbStepHits = CharacterCollisionsSweep(
                                stepCheckStartPos,
                                characterRotation,
                                -characterUp,
                                MaxStepHeight - CollisionOffset,
                                out outerStepHit,
                                _internalCharacterHits,
                                0f,
                                true);

                // Check for overlaps and obstructions at the hit position
                if (CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out tmpCollider))
                {
                    stabilityReport.ValidStepDetected = true;
                    stabilityReport.SteppedCollider = tmpCollider;
                }
            }
        }

        private bool CheckStepValidity(int nbStepHits, Vector3 characterPosition, Quaternion characterRotation, Vector3 innerHitDirection, Vector3 stepCheckStartPos, out Collider hitCollider)
        {
            hitCollider = null;
            Vector3 characterUp = characterRotation * Vector3.up;

            // Find the farthest valid hit for stepping
            bool foundValidStepPosition = false;

            while (nbStepHits > 0 && !foundValidStepPosition)
            {
                // Get farthest hit among the remaining hits
                RaycastHit farthestHit = new RaycastHit();
                float farthestDistance = 0f;
                int farthestIndex = 0;
                for (int i = 0; i < nbStepHits; i++)
                {
                    float hitDistance = _internalCharacterHits[i].distance;
                    if (hitDistance > farthestDistance)
                    {
                        farthestDistance = hitDistance;
                        farthestHit = _internalCharacterHits[i];
                        farthestIndex = i;
                    }
                }

                Vector3 characterPositionAtHit = stepCheckStartPos + (-characterUp * (farthestHit.distance - CollisionOffset));

                int atStepOverlaps = CharacterCollisionsOverlap(characterPositionAtHit, characterRotation, this._internalProbedColliders);
                if (atStepOverlaps <= 0)
                {
                    // Check for outer hit slope normal stability at the step position
                    if (CharacterCollisionsRaycast(
                            farthestHit.point + (characterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal),
                            -characterUp,
                            MaxStepHeight + SecondaryProbesVertical,
                            out RaycastHit outerSlopeHit,
                            _internalCharacterHits,
                            true) > 0)
                    {
                        if (IsStableOnNormal(outerSlopeHit.normal))
                        {
                            // Cast upward to detect any obstructions to moving there
                            if (CharacterCollisionsSweep(
                                                characterPosition, // position
                                                characterRotation, // rotation
                                                characterUp, // direction
                                                MaxStepHeight - farthestHit.distance, // distance
                                                out RaycastHit tmpUpObstructionHit, // closest hit
                                                _internalCharacterHits) // all hits
                                    <= 0)
                            {
                                // Do inner step check...
                                bool innerStepValid = false;
                                RaycastHit innerStepHit;

                                if (AllowSteppingWithoutStableGrounding)
                                {
                                    innerStepValid = true;
                                }
                                else
                                {
                                    // At the capsule center at the step height
                                    if (CharacterCollisionsRaycast(
                                            characterPosition + Vector3.Project((characterPositionAtHit - characterPosition), characterUp),
                                            -characterUp,
                                            MaxStepHeight,
                                            out innerStepHit,
                                            _internalCharacterHits,
                                            true) > 0)
                                    {
                                        if (IsStableOnNormal(innerStepHit.normal))
                                        {
                                            innerStepValid = true;
                                        }
                                    }
                                }

                                if (!innerStepValid)
                                {
                                    // At inner step of the step point
                                    if (CharacterCollisionsRaycast(
                                            farthestHit.point + (innerHitDirection * SecondaryProbesHorizontal),
                                            -characterUp,
                                            MaxStepHeight,
                                            out innerStepHit,
                                            _internalCharacterHits,
                                            true) > 0)
                                    {
                                        if (IsStableOnNormal(innerStepHit.normal))
                                        {
                                            innerStepValid = true;
                                        }
                                    }
                                }

                                // Final validation of step
                                if (innerStepValid)
                                {
                                    hitCollider = farthestHit.collider;
                                    foundValidStepPosition = true;
                                    return true;
                                }
                            }
                        }
                    }
                }

                // Discard hit if not valid step
                if (!foundValidStepPosition)
                {
                    nbStepHits--;
                    if (farthestIndex < nbStepHits)
                    {
                        _internalCharacterHits[farthestIndex] = _internalCharacterHits[nbStepHits];
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Get true linear velocity (taking into account rotational velocity) on a given point of a rigidbody
        /// </summary>
        public void GetVelocityFromRigidbodyMovement(Rigidbody interactiveRigidbody, Vector3 atPoint, float deltaTime, out Vector3 linearVelocity, out Vector3 angularVelocity)
        {
            if (deltaTime > 0f)
            {
                linearVelocity = interactiveRigidbody.velocity;
                angularVelocity = interactiveRigidbody.angularVelocity;
                if(interactiveRigidbody.isKinematic)
                {
                    PhysicsMover physicsMover = interactiveRigidbody.GetComponent<PhysicsMover>();
                    if (physicsMover)
                    {
                        linearVelocity = physicsMover.Velocity;
                        angularVelocity = physicsMover.AngularVelocity;
                    }
                }

                if (angularVelocity != Vector3.zero)
                {
                    Vector3 centerOfRotation = interactiveRigidbody.transform.TransformPoint(interactiveRigidbody.centerOfMass);

                    Vector3 centerOfRotationToPoint = atPoint - centerOfRotation;
                    Quaternion rotationFromInteractiveRigidbody = Quaternion.Euler(Mathf.Rad2Deg * angularVelocity * deltaTime);
                    Vector3 finalPointPosition = centerOfRotation + (rotationFromInteractiveRigidbody * centerOfRotationToPoint);
                    linearVelocity += (finalPointPosition - atPoint) / deltaTime;
                }
            }
            else
            {
                linearVelocity = default;
                angularVelocity = default;
                return;
            }
        }

        /// <summary>
        /// Determines if a collider has an attached interactive rigidbody 
        ///    以下情况返回 true:
        ///       -1- 参数 onCollider 是 rigidbody 且是 PhysicsMover
        ///       -2- 参数 onCollider 是 rigidbody 且是 常规刚体 (物理引擎控制)
        /// </summary>
        private Rigidbody GetInteractiveRigidbody(Collider onCollider) // see__
        {
            Rigidbody colliderAttachedRigidbody = onCollider.attachedRigidbody;
            if (colliderAttachedRigidbody)
            {
                if (colliderAttachedRigidbody.gameObject.GetComponent<PhysicsMover>())
                {
                    return colliderAttachedRigidbody;
                }

                if (!colliderAttachedRigidbody.isKinematic)
                {
                    return colliderAttachedRigidbody;
                }
            }
            return null;
        }


        /// <summary>
        /// Calculates the velocity required to move the character to the target position over a specific deltaTime.
        /// Useful for when you wish to work with positions rather than velocities in the UpdateVelocity callback 
        /// </summary>
        public Vector3 GetVelocityForMovePosition(Vector3 fromPosition, Vector3 toPosition, float deltaTime)
        {
            return GetVelocityFromMovement(toPosition - fromPosition, deltaTime);
        }

        // 计算上一帧 位移速度
        // param: movement: 在 deltaTime 时间内位移的 offset
        public Vector3 GetVelocityFromMovement(Vector3 movement, float deltaTime)
        {
            if (deltaTime <= 0f)
                return Vector3.zero;

            return movement / deltaTime;
        }


        /// <summary>
        /// Trims a vector to make it restricted against a plane 
        /// </summary>
        private void RestrictVectorToPlane(ref Vector3 vector, Vector3 toPlane)
        {
            if (vector.x > 0 != toPlane.x > 0)
            {
                vector.x = 0;
            }
            if (vector.y > 0 != toPlane.y > 0)
            {
                vector.y = 0;
            }
            if (vector.z > 0 != toPlane.z > 0)
            {
                vector.z = 0;
            }
        }

        /// <summary>
        ///     Detect if the character capsule is overlapping with anything collidable; 
        ///     收集所有和 本角色capsule 相交的 colliders, 排除掉其中所有无效的, 将有效元素紧致排列在 overlappedColliders 容器的前部, 最后返回有效元素的个数;
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterCollisionsOverlap(Vector3 position, Quaternion rotation, Collider[] overlappedColliders, float inflate = 0f, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = CollidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = CollidableLayers & StableGroundLayers;
            }

            Vector3 bottom = position + (rotation * _characterTransformToCapsuleBottomHemi); // 角色capsule 下半球球心 posWS 
            Vector3 top = position + (rotation * _characterTransformToCapsuleTopHemi);      // 角色capsule 上半球球心 posWS 
            if (inflate != 0f) // 膨胀
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            int nbHits = 0;
            int nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc( // 检查收集 本capsule 体内的所有碰撞体; 返回碰撞体数量
                        bottom,
                        top,
                        Capsule.radius + inflate,
                        overlappedColliders,
                        queryLayers,
                        QueryTriggerInteraction.Ignore);

            // Filter out invalid colliders
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                //  overlappedColliders 中可能存在数个 无效元素, 且这些无效元素的分布是任意的;  暂将这些无效元素 看作一个空洞
                //  下方算法, 只要找到一个空洞, 且这个空洞后方还有有效元素,  就会把最后一个有效元素, 填入这个空洞
                //  一圈下来, 就能把数组中所有有效元素 都紧致地排到前方 
                if (!CheckIfColliderValidForCollisions(overlappedColliders[i]))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits; 
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterOverlap(Vector3 position, Quaternion rotation, Collider[] overlappedColliders, LayerMask layers, QueryTriggerInteraction triggerInteraction, float inflate = 0f)
        {
            Vector3 bottom = position + (rotation * _characterTransformToCapsuleBottomHemi);
            Vector3 top = position + (rotation * _characterTransformToCapsuleTopHemi);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            int nbHits = 0;
            int nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc(
                        bottom,
                        top,
                        Capsule.radius + inflate,
                        overlappedColliders,
                        layers,
                        triggerInteraction);

            // Filter out the character capsule itself
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                if (overlappedColliders[i] == Capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect collision hits
        ///  把角色 capsule collider 沿着参数 direction cast 一段距离(长度就是 direction 的模), 得到所有 hit;
        ///  去掉所有无效 hit, 将有效 hit 排列在  参数容器 hits 前区;  
        ///  将距离最近的一个 hit实例 存入参数 closestHit; 
        ///  返回 所有有效 hit 的数量;
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterCollisionsSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, float inflate = 0f, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = CollidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = CollidableLayers & StableGroundLayers;
            }

            Vector3 bottom = position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * SweepProbingBackstepDistance);   // 从 角色capsule 下半球球心 posWS, 朝 direction 反方向回撤一小步
            Vector3 top = position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * SweepProbingBackstepDistance);         // 从 角色capsule 上半球球心 posWS, 朝 direction 反方向回撤一小步

            // 适当膨胀一圈
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            // Capsule cast
            int nbHits = 0;
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc( // 返回得到的 hit 数量
                    bottom,
                    top,
                    Capsule.radius + inflate,
                    direction,                              // cast 方向
                    hits,                                   // 收集结果
                    distance + SweepProbingBackstepDistance, // maxDistance 
                    queryLayers,
                    QueryTriggerInteraction.Ignore);

            // Hits filter
            closestHit = new RaycastHit();  // out 参数
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                hits[i].distance -= SweepProbingBackstepDistance;

                RaycastHit hit = hits[i];
                float hitDistance = hit.distance;

                // Filter out the invalid hits
                if (hitDistance <= 0f || !CheckIfColliderValidForCollisions(hit.collider)) // 说明是无效的 hit
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits]; // 用队尾有效元素 前插进这个空洞中;
                    }
                }
                else
                {
                    // Remember closest valid hit -- 收集 距离最近的那个 hit
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, LayerMask layers, QueryTriggerInteraction triggerInteraction, float inflate = 0f)
        {
            closestHit = new RaycastHit();

            Vector3 bottom = position + (rotation * _characterTransformToCapsuleBottomHemi);
            Vector3 top = position + (rotation * _characterTransformToCapsuleTopHemi);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            // Capsule cast
            int nbHits = 0;
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                bottom,
                top,
                Capsule.radius + inflate,
                direction,
                hits,
                distance,
                layers,
                triggerInteraction);

            // Hits filter
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                RaycastHit hit = hits[i];

                // Filter out the character capsule
                if (hit.distance <= 0f || hit.collider == Capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    float hitDistance = hit.distance;
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Casts the character volume in the character's downward direction to detect ground 
        /// 将 角色capsule 向下 cast 一段距离, 找到最近的 hit, 将相关数据回传;
        /// 
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        private bool CharacterGroundSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit) // see__
        {
            closestHit = new RaycastHit();

            // Capsule cast
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * GroundProbingBackstepDistance),
                position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * GroundProbingBackstepDistance),
                Capsule.radius,
                direction,
                _internalCharacterHits,
                distance + GroundProbingBackstepDistance,
                CollidableLayers & StableGroundLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            bool foundValidHit = false;
            float closestDistance = Mathf.Infinity;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                RaycastHit hit = _internalCharacterHits[i];
                float hitDistance = hit.distance;

                // Find the closest valid hit
                if (hitDistance > 0f && CheckIfColliderValidForCollisions(hit.collider))
                {
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestHit.distance -= GroundProbingBackstepDistance;
                        closestDistance = hitDistance;

                        foundValidHit = true;
                    }
                }
            }

            return foundValidHit;
        }


        /// <summary>
        /// Raycasts to detect collision hits 
        /// 用 ray cast 来检查目标方向 目标距离内是否存在 hit
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterCollisionsRaycast(Vector3 position, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = CollidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = CollidableLayers & StableGroundLayers;
            }

            // Raycast
            int nbHits = 0;
            int nbUnfilteredHits = Physics.RaycastNonAlloc(
                position,
                direction,
                hits,
                distance,
                queryLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            closestHit = new RaycastHit();              // out 参数
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                RaycastHit hit = hits[i];
                float hitDistance = hit.distance;

                // Filter out the invalid hits
                if (hitDistance <= 0f ||
                    !CheckIfColliderValidForCollisions(hit.collider))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];  // 用队尾有效元素 前插进这个空洞中;
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }
    }
}
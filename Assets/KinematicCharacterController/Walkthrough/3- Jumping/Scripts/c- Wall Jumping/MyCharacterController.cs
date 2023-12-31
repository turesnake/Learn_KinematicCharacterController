using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.WallJumping
{
    public struct PlayerCharacterInputs
    {
        public float MoveAxisForward;
        public float MoveAxisRight;
        public Quaternion CameraRotation;
        public bool JumpDown;
    }

    public class MyCharacterController : MonoBehaviour, ICharacterController
    {
        public KinematicCharacterMotor Motor;

        [Header("Stable Movement")]
        public float MaxStableMoveSpeed = 10f;
        public float StableMovementSharpness = 15;
        public float OrientationSharpness = 10;

        [Header("Air Movement")]
        public float MaxAirMoveSpeed = 10f;
        public float AirAccelerationSpeed = 5f;
        public float Drag = 0.1f;

        [Header("Jumping")]
        public bool AllowJumpingWhenSliding = false;
        public bool AllowDoubleJump = false;
        public bool AllowWallJump = false;
        public float JumpSpeed = 10f;


        // the extra time before landing where you can press jump and it’ll still jump once you land 
        // -- 在你落地前的 N秒, 在此期间按下的跳跃 依然有效, 会在你落地后触发跳跃
        public float JumpPreGroundingGraceTime = 0f;  //  (Grace Time 宽限时间)


        // the extra time after leaving stable ground where you’ll still be allowed to jump 
        // -- 在你离开 稳定地面 后的 N秒内, 你依然可以触发跳跃
        public float JumpPostGroundingGraceTime = 0f;


        [Header("Misc")]
        public Vector3 Gravity = new Vector3(0, -30f, 0);
        public Transform MeshRoot;


        private Vector3 _moveInputVector; // 玩家输入的 本帧 前进方向 (xz平面版)  归一化了
        private Vector3 _lookInputVector; // 相机的 本帧 观察方向 (xz平面版) 归一化了


        // 玩家按下跳跃键, 发出的 跳跃指令, 此值被设为 true, 然后等待被处理:
        // -1- 当前 角色在 斜坡/稳定地面 上 这种可跳状态; 或 角色刚离开 斜坡/稳定地面 一小段时间, 依然支持跳;  此时代码会接手处理这个跳跃指令, 立马将它置 false
        // -2- 当前 角色并不满足 -1- 中的条件, 无法处理这次跳跃; 此值会被留置, 若未来几帧玩家再次进入 可跳状态, 还是能继续接手处理此指令;  但若等待时间过久, 此指令就会被销毁
        private bool _jumpRequested = false;

        private bool _jumpConsumed = false; // 跳跃消耗, 其实就是 计时器 = 1, false 表示没跳, true 表示跳了一次 (以此来支持 二段跳) 
        private bool _jumpedThisFrame = false; // 本帧是否 起跳了
        private float _timeSinceJumpRequested = Mathf.Infinity; // 玩家发出跳跃指令后, 开始计时; 直到后面某一帧玩家接手处理这个指令, 或因为等太久了这个指令被清理掉

        
        private float _timeSinceLastAbleToJump = 0f; // 只要处于可跳状态, 本值始终为 0, 脱离可跳状态后, 此值开始计时, 直到下次再次进入可跳区域
        private bool _doubleJumpConsumed = false;
        private bool _canWallJump = false; // 是否进入 可以墙跳的状态
        private Vector3 _wallJumpNormal;




        private void Start()
        {
            // Assign to motor
            Motor.CharacterController = this;
        }

        /// <summary>
        /// This is called every frame by MyPlayer in order to tell the character what its inputs are
        ///  在 excute order = 0 的 Update() 中被调用, 保证一定在 K Update() 之前;
        /// </summary>
        public void SetInputs(ref PlayerCharacterInputs inputs)
        {
            // Clamp input
            Vector3 moveInputVector = Vector3.ClampMagnitude(new Vector3(inputs.MoveAxisRight, 0f, inputs.MoveAxisForward), 1f);

            // Calculate camera direction and rotation on the character plane
            Vector3 cameraPlanarDirection = Vector3.ProjectOnPlane(inputs.CameraRotation * Vector3.forward, Motor.CharacterUp).normalized;
            if (cameraPlanarDirection.sqrMagnitude == 0f)
            {
                cameraPlanarDirection = Vector3.ProjectOnPlane(inputs.CameraRotation * Vector3.up, Motor.CharacterUp).normalized;
            }
            Quaternion cameraPlanarRotation = Quaternion.LookRotation(cameraPlanarDirection, Motor.CharacterUp);

            // Move and look inputs
            _moveInputVector = cameraPlanarRotation * moveInputVector;
            _lookInputVector = cameraPlanarDirection;

            // Jumping input
            if (inputs.JumpDown)
            {
                _timeSinceJumpRequested = 0f;
                _jumpRequested = true;
            }
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called before the character begins its movement update
        /// </summary>
        public void BeforeCharacterUpdate(float deltaTime)
        {
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its rotation should be right now. 
        /// This is the ONLY place where you should set the character's rotation
        /// </summary>
        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            if (_lookInputVector != Vector3.zero && OrientationSharpness > 0f)
            {
                // Smoothly interpolate from current to target look direction
                Vector3 smoothedLookInputDirection = Vector3.Slerp(Motor.CharacterForward, _lookInputVector, 1 - Mathf.Exp(-OrientationSharpness * deltaTime)).normalized;

                // Set the current rotation (which will be used by the KinematicCharacterMotor)
                currentRotation = Quaternion.LookRotation(smoothedLookInputDirection, Motor.CharacterUp);
            }
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is where you tell your character what its velocity should be right now. 
        /// This is the ONLY place where you can set the character's velocity
        /// </summary>
        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            Vector3 targetMovementVelocity = Vector3.zero;
            if (Motor.GroundingStatus.IsStableOnGround)
            {
                // Reorient velocity on slope
                currentVelocity = Motor.GetDirectionTangentToSurface(currentVelocity, Motor.GroundingStatus.GroundNormal) * currentVelocity.magnitude;

                // Calculate target velocity
                Vector3 inputRight = Vector3.Cross(_moveInputVector, Motor.CharacterUp);
                Vector3 reorientedInput = Vector3.Cross(Motor.GroundingStatus.GroundNormal, inputRight).normalized * _moveInputVector.magnitude;
                targetMovementVelocity = reorientedInput * MaxStableMoveSpeed;

                // Smooth movement Velocity
                currentVelocity = Vector3.Lerp(currentVelocity, targetMovementVelocity, 1 - Mathf.Exp(-StableMovementSharpness * deltaTime));
            }
            else
            {
                // Add move input
                if (_moveInputVector.sqrMagnitude > 0f)
                {
                    targetMovementVelocity = _moveInputVector * MaxAirMoveSpeed;

                    // Prevent climbing on un-stable slopes with air movement
                    if (Motor.GroundingStatus.FoundAnyGround)
                    {
                        Vector3 perpenticularObstructionNormal = Vector3.Cross(Vector3.Cross(Motor.CharacterUp, Motor.GroundingStatus.GroundNormal), Motor.CharacterUp).normalized;
                        targetMovementVelocity = Vector3.ProjectOnPlane(targetMovementVelocity, perpenticularObstructionNormal);
                    }

                    Vector3 velocityDiff = Vector3.ProjectOnPlane(targetMovementVelocity - currentVelocity, Gravity);
                    currentVelocity += velocityDiff * AirAccelerationSpeed * deltaTime;
                }

                // Gravity
                currentVelocity += Gravity * deltaTime;

                // Drag
                currentVelocity *= (1f / (1f + (Drag * deltaTime)));
            }

            // Handle jumping
            {
                _jumpedThisFrame = false;
                _timeSinceJumpRequested += deltaTime;
                if (_jumpRequested)
                {
                    // Handle double jump
                    if (AllowDoubleJump)
                    {
                        if (_jumpConsumed && !_doubleJumpConsumed && (AllowJumpingWhenSliding ? !Motor.GroundingStatus.FoundAnyGround : !Motor.GroundingStatus.IsStableOnGround))
                        {
                            Motor.ForceUnground(0.1f);

                            // Add to the return velocity and reset jump state
                            currentVelocity += (Motor.CharacterUp * JumpSpeed) - Vector3.Project(currentVelocity, Motor.CharacterUp);
                            _jumpRequested = false;
                            _doubleJumpConsumed = true;
                            _jumpedThisFrame = true;
                        }
                    }

                    // See if we actually are allowed to jump
                    if (_canWallJump ||
                        ( 
                            !_jumpConsumed && 
                            ( 
                                (AllowJumpingWhenSliding ? Motor.GroundingStatus.FoundAnyGround : Motor.GroundingStatus.IsStableOnGround) ||  // 要么在斜坡上滑动的时候跳, 要么在地面上跳
                                _timeSinceLastAbleToJump <= JumpPostGroundingGraceTime // 在你离开 (斜坡/稳定地面) 的一段时间内, 你依然可以触发跳跃
                            ) 
                        )
                    )
                    {
                        // Calculate jump direction before ungrounding
                        Vector3 jumpDirection = Motor.CharacterUp;
                        if (_canWallJump)
                        {
                            jumpDirection = _wallJumpNormal;
                        }
                        else if (Motor.GroundingStatus.FoundAnyGround && !Motor.GroundingStatus.IsStableOnGround) // 陡峭的斜坡
                        {
                            jumpDirection = Motor.GroundingStatus.GroundNormal;
                        }

                        // Makes the character skip ground probing/snapping on its next update. 
                        // If this line weren't here, the character would remain snapped to the ground when trying to jump. Try commenting this line out and see.
                        Motor.ForceUnground(0.1f);

                        // Add to the return velocity and reset jump state
                        // 减掉的是 沿着 CharacterUp 轴线的运动分量; 
                        // 或者说, 先让原运动 在 CharacterUp 方向的分量归零,  然后再叠加 跳跃速度
                        currentVelocity += (jumpDirection * JumpSpeed) - Vector3.Project(currentVelocity, Motor.CharacterUp);
                        _jumpRequested = false;
                        _jumpConsumed = true;
                        _jumpedThisFrame = true;
                    }
                }

                // Reset wall jump
                _canWallJump = false;
            }
        }

        /// <summary>
        /// (Called by KinematicCharacterMotor during its update cycle)
        /// This is called after the character has finished its movement update
        /// </summary>
        public void AfterCharacterUpdate(float deltaTime)
        {
            // Handle jump-related values
            {
                // Handle jumping pre-ground grace period
                // 如果玩家发起跳跃指令后, 角色不在可跳状态, 这个指令就会被留置; 在之后的数帧内, 只要角色回到可跳状态, 都能回来继续处理这个指令;
                // 但时间一旦超出界限, 这个跳跃指令就会被清除
                if (_jumpRequested && _timeSinceJumpRequested > JumpPreGroundingGraceTime)
                {
                    _jumpRequested = false;
                }

                if (AllowJumpingWhenSliding ? Motor.GroundingStatus.FoundAnyGround : Motor.GroundingStatus.IsStableOnGround) 
                {
                    // ----- 要么在斜坡上滑动, 要么在地面上  -----
                    // 这些场合都支持跳跃;  只要某一帧符合这些条件, 就会将 跳跃相关的数据重置:

                    // If we're on a ground surface, reset jumping values
                    if (!_jumpedThisFrame) // 本帧不在跳
                    {
                        _doubleJumpConsumed = false;
                        _jumpConsumed = false;
                    }
                    _timeSinceLastAbleToJump = 0f; // 已经准备好 迎接跳跃指令了
                }
                else
                {
                    // ----- 在空中 -----

                    // Keep track of time since we were last able to jump ( for grace period 为了配合 JumpPostGroundingGraceTime 的使用 )
                    _timeSinceLastAbleToJump += deltaTime;
                }
            }
        }

        public bool IsColliderValidForCollisions(Collider coll)
        {
            return true;
        }

        public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
        {
        }

        public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
        {
            // We can wall jump only if we are not stable on ground and are moving against an obstruction (在障碍物上移动)
            if (AllowWallJump && !Motor.GroundingStatus.IsStableOnGround && !hitStabilityReport.IsStable)
            {
                _canWallJump = true;
                _wallJumpNormal = hitNormal;
            }
        }

        public void PostGroundingUpdate(float deltaTime)
        {
        }

        public void AddVelocity(Vector3 velocity)
        {
        }

        public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
        {
        }

        public void OnDiscreteCollisionDetected(Collider hitCollider)
        {
        }
    }
}
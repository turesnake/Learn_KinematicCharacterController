using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KinematicCharacterController
{
    public interface ICharacterController
    {
        /// <summary>
        /// This is called when the motor wants to know what its rotation should be right now
        /// </summary>
        void UpdateRotation(ref Quaternion currentRotation, float deltaTime);   // !!! UpdatePhase=2 -4- 


        /// <summary>
        /// This is called when the motor wants to know what its velocity should be right now
        /// </summary>
        void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime);      // !!! UpdatePhase=2 -5- 


        /// <summary>
        /// This is called before the motor does anything; 此时 motor 只对部分变量做了初始化
        /// </summary>
        void BeforeCharacterUpdate(float deltaTime);    // !!! UpdatePhase=1 -1- 


        /// <summary>
        /// This is called after the motor has finished its ground probing, but before PhysicsMover/Velocity/etc.... handling
        /// </summary>
        void PostGroundingUpdate(float deltaTime);      // !!! UpdatePhase=1 -3- 


        /// <summary>
        /// This is called after the motor has finished everything in its update
        /// </summary>
        void AfterCharacterUpdate(float deltaTime);     // !!! UpdatePhase=2 -7- 


        /// <summary>
        /// This is called after when the motor wants to know if the collider can be collided with (or if we just go through it)
        /// </summary>
        bool IsColliderValidForCollisions(Collider coll);


        /// <summary>
        /// This is called when the motor's ground probing detects a ground hit
        /// </summary>
        void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport);     // !!! UpdatePhase=1 -2- 


        /// <summary>
        /// This is called when the motor's movement logic detects a hit
        /// </summary>
        void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport);   // !!! UpdatePhase=1 / UpdatePhase2


        /// <summary>
        /// This is called after every move hit, to give you an opportunity to modify the HitStabilityReport to your liking
        /// </summary>
        void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport); // !!! UpdatePhase=1 /


        /// <summary>
        /// This is called when the character detects discrete collisions (collisions that don't result from the motor's capsuleCasts when moving) 
        ///  通过此函数, 用户能知道每次 update() 结束时, 角色紧密相挨着的 collider 有哪些, (比如地面,墙面等)
        ///  -----
        ///  细节:
        ///  在每次 update() 的最后时刻, 角色完成了旋转和移动, 在最终pos上, 再做一次 capsule overlaps 检测, 同时把 充气值做大点;
        ///  这样就能把所有和 角色紧密贴在一起的 collider 检测出来, 比如地面, 墙面等;
        ///  在每一帧, 每检测出一个这样的 collider, 本函数就会被调用一次;
        ///  
        /// </summary>
        void OnDiscreteCollisionDetected(Collider hitCollider);     // !!! UpdatePhase=2 -6- 
    }
}
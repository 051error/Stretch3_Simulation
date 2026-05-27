import numpy as np
import mujoco
import math
from stretch_sim.navigation import NavigationController
class IKSolver:
    def __init__(self, model, data,logger=None):
        self.model = model
        self.data = data
        self.nav_controller = NavigationController()
        self.logger=logger

    def compute_ik(self, target_pos, max_iter=100, tol=0.01):

        target_pos = np.array(target_pos)
        lift_nv = 8
        arm_l3_nv = 9
        arm_l2_nv = 10
        arm_l1_nv = 11
        arm_l0_nv = 12
        wrist_yaw_nv = 13

        ik_data = mujoco.MjData(self.model)
        ik_data.qpos[:] = self.data.qpos.copy()

    #    print("\n========== IK START ==========")
      #  print("Initial qpos:")
     #   print(f"  lift        : {ik_data.qpos[9]}")
      #  print(f"  arm joints  : {ik_data.qpos[10:14]}")
       # print(f"  wrist yaw   : {ik_data.qpos[14]}")
     #   print(f"Target pos    : {target_pos}")
       # print("================================\n")

        ee_site_id = self.model.site('ee_site').id
        
        print(f"Using site: ee_site (id={ee_site_id})")

        for i in range(max_iter):
            print(f"\n------ ITERATION {i} ------")

            mujoco.mj_forward(self.model, ik_data)

            # Get end-effector position from SITE
            ee_pos = ik_data.site_xpos[ee_site_id].copy()
            error = target_pos - ee_pos
            error_norm = np.linalg.norm(error)

          #  print(f"EE position (ee_site)        : {ee_pos}")
           # print(f"Target position              : {target_pos}")
           # print(f"Error vector                 : {error}")
           # print(f"Error norm                   : {error_norm}")

            if error_norm < tol:
                print("\n✓ IK CONVERGED")
                print(f"Iterations    : {i}")
                print(f"Final error   : {error_norm}")
                print(f"Final ee_pos  : {ee_pos}")
                
                return True, np.array([
                    ik_data.qpos[9],   # lift
                    ik_data.qpos[10],  # arm_l3
                    ik_data.qpos[11],  # arm_l2
                    ik_data.qpos[12],  # arm_l1
                    ik_data.qpos[13],  # arm_l0
                    ik_data.qpos[14]   # wrist_yaw
                ])
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, ik_data, jacp, jacr, ee_site_id)
            
            J = np.column_stack([
                jacp[:3, lift_nv],
                jacp[:3, arm_l3_nv],
                jacp[:3, arm_l2_nv],
                jacp[:3, arm_l1_nv],
                jacp[:3, arm_l0_nv],
                jacp[:3, wrist_yaw_nv]
            ])

            #print("Jacobian J:")
            #print(J)

           
            damping = 0.01
            JJt = J @ J.T
            inv_term = np.linalg.inv(JJt + damping * np.eye(3))
            J_pinv = J.T @ inv_term
            dq = 0.1 * J_pinv @ error

            print("dq:")
            print(dq)

            print("qpos BEFORE update:")
            print(ik_data.qpos[9:15])

            ik_data.qpos[9]  += dq[0]  # lift
            ik_data.qpos[10] += dq[1]  # arm_l3
            ik_data.qpos[11] += dq[2]  # arm_l2
            ik_data.qpos[12] += dq[3]  # arm_l1
            ik_data.qpos[13] += dq[4]  # arm_l0
            ik_data.qpos[14] += dq[5]  # wrist_yaw

            print("qpos AFTER update:")
            print(ik_data.qpos[9:15])

        print("\n✗ IK FAILED")
        print(f"Max iterations reached: {max_iter}")
        print("Final qpos:")
        print(ik_data.qpos[9:15])
        print(f"Final ee_pos: {ik_data.site_xpos[ee_site_id]}")

        return False, np.array([
            ik_data.qpos[9],
            ik_data.qpos[10],
            ik_data.qpos[11],
            ik_data.qpos[12],
            ik_data.qpos[13],
            ik_data.qpos[14]
        ])
    
    def align_with_target(self,pos,quat,tomato_name='tomato1'):

        current_yaw = self.nav_controller._quaternion_to_yaw(quat)
        '''fruit_body_id = mujoco.mj_name2id(
            self.model, 
            mujoco.mjtObj.mjOBJ_BODY, 
            'tomato2'
        )
        fruit_pos = self.data.xpos[fruit_body_id].copy()'''
        # Get the site instead of body
        fruit_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,  f'{tomato_name}_site')

        if fruit_site_id >= 0:
            fruit_pos = self.data.site_xpos[fruit_site_id].copy()
            #self.get_logger().info(f'Actual fruit site position: {fruit_pos}')
        else:
            
            fruit_pos = np.array([-1.0, 5.54, 1.05])
        dx = fruit_pos[0] - pos[0]
        dy = fruit_pos[1] - pos[1]
        if self.logger:
            self.logger.info(f"dx: {dx}, dy: {dy}")
        desired_yaw = math.atan2(dy, dx)
        yaw_diff = (desired_yaw+1.5708) - current_yaw
        yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        return yaw_diff, current_yaw, desired_yaw
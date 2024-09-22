import tinyik
import numpy as np
import pyFileInteraction as fi

class simIK() :
    def __init__(self, titf_json_path = None) -> None :
        self.titf = None
        if titf_json_path != None :
            self.load_titf(titf_json_path)
        else :
            pass
        
    def create_titf(self, name : str, titf : list, joint_limit : list = [], pos_limit : list = [], unit : str = "mm") -> None :
        """Create Manipurator JSON configuration file.

        Args:
            name (str): Name of the file.
            titf (list): Tranfer model of manipurator (similar to URDF).
            joint_limit (list): limit of each joint. Defaults to "[]". When Defaults, limit of each joint will set to [-1e12, 1e12].
            pos_limit (list): limit of work space. Defaults to "[]". When Defaults, limit of each position will set to [-1e12, 1e12].
            unit (str, optional): unit that you want to describe. did not effect on calculation, but all value on TITF must be the same unit. Defaults to "mm".
        
        TITF describe:
            TITF is describe by 2 setting
            \tshift position (list): position shift from the previous position.
            \trotate axit (str): rotate axit of the manipurator on the position from last shift position.
            
        TITF Example:
            titf = [
                [0, 0, 65],
                "z", [0, 0, 50],
                "x", [0, -250, 41],
                "x", [0, 450, 0]]
        """
        
    def load_titf(self, titf_json_path) :
        data = fi.read_json(titf_json_path)
        titf_name = data["name"]
        unit = data["unit"]
        self.titf = list(data["titf"])
        self.joint_limit = data["joint_limit"]
        joint_num = self.titf.count("x") + self.titf.count("y") + self.titf.count("z")
        if len(self.joint_limit) != joint_num :
            e = f"Warning : TITF joint limit quantity are not matched to quantity of joint that exit. {joint_num} joint exit ; {len(self.joint_limit)} limit exit"
            raise Exception(e)
        for i, j_lim in enumerate(self.joint_limit) :
            if len(j_lim) != 2 :
                e = f"Warning : each TITF joint limit are must contain 2 limit [min, max] ; {len(j_lim)} limit exit in joint number {i+1}"
                raise Exception(e)
        self.position_limit = data["pos_limit"]
        if len(self.position_limit) != 3 :
            e = f"Warning : TITF Position limit must contain 3 axit [x, y, z] ; {len(self.position_limit)} Position limit exit"
            raise Exception(e)
        for i, p_lim in enumerate(self.position_limit) :
            if len(p_lim) != 2 :
                axit = "x" if i == 0 else "y" if i == 1 else "z"
                e = f"Warning : each TITF position limit are must contain 2 limit [min, max] ; {len(j_lim)} limit exit on {axit} axit."
                raise Exception(e)
        self.arm = tinyik.Actuator(self.titf, optimizer=tinyik.ScipySmoothOptimizer())
        return titf_name, unit
        
    def IK(self, target : list, ignore_limit_position = False, ignore_limit_joint = False) :
        self.err_check()
        if not ignore_limit_position :
            for i, tar in enumerate(target) :
                if tar < min(self.position_limit[i]) or tar > max(self.position_limit[i]) :
                    axit = "x" if i == 0 else "y" if i == 1 else "z"
                    e = f"Warning : Position Out range on {axit} axit : {tar} ; Limit : {self.position_limit[i]}"
                    raise Exception (e)
        self.arm.ee = target
        raw_list_deg = list(np.rad2deg(self.arm.angles))
        list_deg = []
        for i, deg in enumerate(raw_list_deg) :
            round_deg = round(deg, 2)
            if not ignore_limit_joint and (round_deg < min(self.joint_limit[i]) or round_deg > max(self.joint_limit[i])) :
                e = f"Warning : Joint Out range on Joint {i+1} : {round_deg} ; Limit : {self.joint_limit[i]}"
                raise Exception (e)
            list_deg.append(round_deg)
        ee_fk_check_list = self.FK(list_deg)
        for i, ee_pos in enumerate(ee_fk_check_list) :
            if round(target[i], 0) != round(ee_pos, 0) :
                axit = "x" if i == 0 else "y" if i == 1 else "z"
                e = f"Warning : IKEE and FKEE Unmatched. IKEE : {ee_fk_check_list} ; FKEE : {target}"
                raise Exception (e)
        else :
            return list_deg
    
    def FK(self, deg : list) -> list :
        self.err_check()
        self.arm.angles = np.deg2rad(deg)
        return list(self.arm.ee)
        
    def err_check(self) -> None :
        if self.titf == None :
            e = "TITF is not declare. Please call 'load_titf' or 'create_titf' before using IK or FK"
            raise Exception(e)
        
# if __name__ == "__main__" :
#     import dynamixelControler as dyc
#     import time
    
#     ik = simIK(titf_json_path="./titf_json/titf1.json")
#     dy = dyc.dyControl("COM5")
#     time.sleep(1)
#     dy.move_synchrony_motors([90, 100, 105, 90], 5)
    
#     time.sleep(10)
    
#     x = 0
#     y = 400
#     z = 200
    
#     target = [x, y, z + 100] # x:side y:front-back z:up-down
#     j1, j2, j3 = ik.IK(target)
#     print(j1 + 90, -j2, j3)
    
#     dy.move_synchrony_motors([j1 + 90, -j2, j3], 5)
    
#     time.sleep(10)
    
#     x = 0
#     y = 600
#     z = 200
    
#     target = [x, y, z + 100] # x:side y:front-back z:up-down
#     j1, j2, j3 = ik.IK(target)
#     print(j1 + 90, -j2, j3)
    
#     dy.move_synchrony_motors([j1 + 90, -j2, j3], 5)

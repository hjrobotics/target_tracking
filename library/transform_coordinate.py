
# coding: utf-8

# In[ ]:


from quaternion import quaternion
class transform_coordinate:
    # l0 is the base location (x0, y0, z0) of the new coordinate based on the old one
    # ori0 is the quaternion of the new coordinate in old coordinate
    def __init__ (self, l0, ori0, rot):
        assert len(l0) == 3
        assert isinstance(ori0, quaternion)
        assert isinstance(rot, quaternion)
        self.l0 = l0
        self.ori0 = ori0
        self.rot = rot
    
    def to_new(self, l, ori):
        assert len(l) == 3
        assert isinstance(ori, quaternion)
        
        rot_neg = self.rot.conjugate()
        
        dl_old_cor = [x-x0 for x, x0 in zip(l, self.l0)]
        dl_new_cor = rot_neg.rotate_vector(dl_old_cor)
        
        dori_old_cor = ori.delta(self.ori0)
        dori_old_cor_aangle = dori_old_cor.to_axis_angle()
        dori_new_cor_aangle = rot_neg.rotate_vector(dori_old_cor_aangle)
        dori_new_cor = quaternion.axis_angle_to_quaternion(dori_new_cor_aangle)
        
        return dl_new_cor, dori_new_cor
    
    def to_old(self, l, ori):
        assert len(l) == 3
        assert isinstance(ori, quaternion)
        
        dl_old_cor = self.rot.rotate_vector(l)
        l_old_cor = [x+x0 for x, x0 in zip(dl_old_cor, self.l0)]
        
        ori_aangle = ori.to_axis_angle()
        dori_old_cor_aangle = self.rot.rotate_vector(ori_aangle)
        dori_old_cor = quaternion.axis_angle_to_quaternion(dori_old_cor_aangle)
        ori_old_cor = dori_old_cor.multiply(self.ori0)
        
        return l_old_cor, ori_old_cor


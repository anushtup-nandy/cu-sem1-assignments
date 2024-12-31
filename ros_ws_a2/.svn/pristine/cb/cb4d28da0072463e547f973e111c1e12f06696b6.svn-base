#!/usr/bin/env python3
# coding: UTF-8
import sys
l11_opy_ = sys.version_info [0] == 2
l1l1l_opy_ = 2048
l11l1ll_opy_ = 7
def l1l1ll_opy_ (l1l11_opy_):
    global ll_opy_
    l1l1_opy_ = ord (l1l11_opy_ [-1])
    l1lll1_opy_ = l1l11_opy_ [:-1]
    l111ll_opy_ = l1l1_opy_ % len (l1lll1_opy_)
    l1111l_opy_ = l1lll1_opy_ [:l111ll_opy_] + l1lll1_opy_ [l111ll_opy_:]
    if l11_opy_:
        l11ll11_opy_ = l1l1lll_opy_ () .join ([l1lll_opy_ (ord (char) - l1l1l_opy_ - (l1ll_opy_ + l1l1_opy_) % l11l1ll_opy_) for l1ll_opy_, char in enumerate (l1111l_opy_)])
    else:
        l11ll11_opy_ = str () .join ([chr (ord (char) - l1l1l_opy_ - (l1ll_opy_ + l1l1_opy_) % l11l1ll_opy_) for l1ll_opy_, char in enumerate (l1111l_opy_)])
    return eval (l11ll11_opy_)
import sys
sys.path.append(l1l1ll_opy_ (u"ࠦ࠳ࠨࠀ"))
import numpy
import rclpy
import transforms3d._gohlketransforms as tf
from .l11llll_opy_ import l1l11l_opy_
import time
def l1lllll_opy_():
   l1l11ll_opy_ = (0.3, 0.2, 0.5)
   l1ll111_opy_ = (0.0, 0.0, 1, 0.0)
   l1lll11_opy_ = (0.3, 0.2, 1.0)
   l1l111l_opy_ = (0.0, 0.0, 0.0, 1.0)
   l1ll11_opy_ = (0.3, 0.1, 0.8)
   l1ll1ll_opy_ = (0.87, 0.0, 0.47, 0.0)
   l11l11_opy_ = (-0.3, 0.3, 0.2)
   l1llll1_opy_ = (0.87, 0.0, 0.47, 0.0)
   return (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_)
def l11lll1_opy_():
   l1l11ll_opy_ = (-0.4, 0.4, 0.1)
   l1ll111_opy_ = (-0.03218073733996252, -0.014727757546290993, -0.415886141536534, -0.9087277978469505)
   l1lll11_opy_ = (-0.4, 0.4, 0.1)
   l1l111l_opy_ = (0.7, 0.5, 0.5, -0.3)
   l1ll11_opy_ = (-0.1, 0.6, 0.4)
   l1ll1ll_opy_ = (0.0, 0.7, 0.6, -0.3)
   l11l11_opy_ = (-0.3, -0.2, 0.3)
   l1llll1_opy_ = (-0.1, 0.7, 0.6, -0.3)
   return (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_)
def l1ll11l_opy_(robot, l1_opy_):
    l1l1ll1_opy_ = 0
    if robot == l1l1ll_opy_ (u"ࠬࡻࡲࠨࠁ"): (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_) = l11lll1_opy_()
    else: (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_) = l1lllll_opy_()
    l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠨࡐࡦࡴࡩࡳࡷࡳࡩ࡯ࡩࠣࡍࡐࠨࠂ"))
    l111_opy_ = tf.translation_matrix(l1l11ll_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1ll111_opy_)
    l1l1ll1_opy_ += l1_opy_.l1lll1l_opy_(l1l1ll_opy_ (u"ࠢࡊࡍ࠴ࠦࠃ"), numpy.dot(l111_opy_,l1111_opy_), 10)
    l111_opy_ = tf.translation_matrix(l1lll11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1l111l_opy_)
    l1l1ll1_opy_ += l1_opy_.l1lll1l_opy_(l1l1ll_opy_ (u"ࠣࡋࡎ࠶ࠧࠄ"), numpy.dot(l111_opy_,l1111_opy_), 10)
    l111_opy_ = tf.translation_matrix(l1ll11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1ll1ll_opy_)
    l1l1ll1_opy_ += l1_opy_.l1lll1l_opy_(l1l1ll_opy_ (u"ࠤࡌࡏ࠸ࠨࠅ"), numpy.dot(l111_opy_,l1111_opy_), 10)
    l111_opy_ = tf.translation_matrix(l11l11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1llll1_opy_)
    l1l1ll1_opy_ += l1_opy_.l1lll1l_opy_(l1l1ll_opy_ (u"ࠥࡍࡐ࠺ࠢࠆ"), numpy.dot(l111_opy_,l1111_opy_), 10)
    return l1l1ll1_opy_
def l11ll1l_opy_(robot, l1_opy_):
    l1llll_opy_ = 0
    if robot == l1l1ll_opy_ (u"ࠫࡺࡸࠧࠇ"): (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_) = l11lll1_opy_()
    else: (l1l11ll_opy_, l1ll111_opy_, l1lll11_opy_, l1l111l_opy_, l1ll11_opy_, l1ll1ll_opy_, l11l11_opy_, l1llll1_opy_) = l1lllll_opy_()
    l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"࡚ࠧࡥࡴࡶ࡬ࡲ࡬ࠦࡣࡢࡴࡷࡩࡸ࡯ࡡ࡯ࠢࡦࡳࡳࡺࡲࡰ࡮ࠥࠈ"))
    l111_opy_ = tf.translation_matrix(l1l11ll_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1ll111_opy_)
    l1llll_opy_ += l1_opy_.l11111_opy_(l1l1ll_opy_ (u"ࠨࡃࡄ࠳ࠥࠉ"), numpy.dot(l111_opy_,l1111_opy_), False, 0.0, 10, 1.5)		#l111l_opy_ l1ll1l_opy_
    l111_opy_ = tf.translation_matrix(l1lll11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1l111l_opy_)
    l1llll_opy_ += l1_opy_.l11111_opy_(l1l1ll_opy_ (u"ࠢࡄࡅ࠵ࠦࠊ"), numpy.dot(l111_opy_,l1111_opy_), False, 0.0, 10, 1.5)		#l1ll1_opy_ l1ll1l_opy_
    l111_opy_ = tf.translation_matrix(l1ll11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1ll1ll_opy_)
    l1llll_opy_ += l1_opy_.l11111_opy_(l1l1ll_opy_ (u"ࠣࡅࡆ࠷ࠧࠋ"), numpy.dot(l111_opy_,l1111_opy_), False, 0.0, 10, 2)		#l111l_opy_ and l1l11l1_opy_
    #print(l1l1ll_opy_ (u"ࠤࡐࡳࡻ࡯࡮ࡨࠢࡶࡩࡨࡵ࡮ࡥࡣࡵࡽࠥࡵࡢ࡫ࡧࡦࡸ࡮ࡼࡥࠣࠌ")
    l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠥࡘࡪࡹࡴࡪࡰࡪࠤࡨࡧࡲࡵࡧࡶ࡭ࡦࡴࠠࡤࡱࡱࡸࡷࡵ࡬ࠡࡹ࡬ࡸ࡭ࠦࡳࡦࡥࡲࡲࡩࡧࡲࡺࠢࡲࡦ࡯࡫ࡣࡵ࡫ࡹࡩࠧࠍ"))
    l111_opy_ = tf.translation_matrix(l1ll11_opy_)
    l1111_opy_ = tf.l1l1111_opy_(l1ll1ll_opy_)
    l11l1l_opy_ = l1_opy_.l11111_opy_(l1l1ll_opy_ (u"ࠦࡘࡕ࠱ࠣࠎ"), numpy.dot(l111_opy_,l1111_opy_), True, 1.5, 20, 2.5)		#l111l1_opy_ l1l1l11_opy_, l11l_opy_ l1ll1l1_opy_
    return l1llll_opy_, l11l1l_opy_
def main(args=None):
    time.sleep(1.0)
    rclpy.init(args=args)
    l1_opy_ = l1l11l_opy_()
    l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠧࡏ࡮ࡪࡶ࡬ࡥࡱ࡯ࡺࡢࡶ࡬ࡳࡳࠦࡤࡰࡰࡨࠦࠏ"))
    robot = l1_opy_.robot.name
    time.sleep(1.0)
    l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠨࡒࡦࡵࡨࡸࡹ࡯࡮ࡨࠢࡵࡳࡧࡵࡴࠣࠐ"))
    l1_opy_.l11ll_opy_()
    time.sleep(1.0)
    l1l_opy_ = 0
    l1l1ll1_opy_ = l1ll11l_opy_(robot, l1_opy_)
    if l1l1ll1_opy_ ==4:
        l1l_opy_+=2.5
        l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠢࡊࡍ࠽ࠤࡕࡇࡓࡔࡇࡇࠦࠑ"))
    else:
        l1l111_opy_ = l1ll11l_opy_(robot, l1_opy_)
        if l1l111_opy_ ==4:
            l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠣࡋࡎ࠾ࠥࡖࡁࡔࡕࡈࡈࠧࠒ"))
            l1l_opy_+=2.5
        else: l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠤࡌࡏࠥࡌࡁࡊࡎࡈࡈࠧࠓ"))
    time.sleep(1.0)
    l1_opy_.l11ll_opy_()
    time.sleep(1.0)
    l1llll_opy_, l11l1l_opy_ = l11ll1l_opy_(robot, l1_opy_)
    if l1llll_opy_ == 5:
        l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠥࡇࡦࡸࡴࡦࡵ࡬ࡥࡳࠦࡣࡰࡰࡷࡶࡴࡲ࠺ࠡࡒࡄࡗࡘࡋࡄࠣࠔ"))
    else:
        l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠦࡈࡧࡲࡵࡧࡶ࡭ࡦࡴࠠࡤࡱࡱࡸࡷࡵ࡬࠻ࠢࡉࡅࡎࡒࡅࡅࠤࠕ"))
        if l1llll_opy_ == 0:
            l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠧࡈࡏࡕࡊࠣࡶࡴࡺࡡࡵ࡫ࡲࡲࠥࡧ࡮ࡥࠢࡷࡶࡦࡴࡳ࡭ࡣࡷ࡭ࡴࡴࠠࡧࡣ࡬ࡰࡪࡪࠢࠖ"))
        elif l1llll_opy_ == 3:
            l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠨࡓࡊࡏࡘࡐ࡙ࡇࡎࡆࡑࡘࡗࠥࡸ࡯ࡵࡣࡷ࡭ࡴࡴࠠࡢࡰࡧࠤࡹࡸࡡ࡯ࡵ࡯ࡥࡹ࡯࡯࡯ࠢࡩࡥ࡮ࡲࡥࡥࠤࠗ"))
        else:
            l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠢࡆࡋࡗࡌࡊࡘࠠࡳࡱࡷࡥࡹ࡯࡯࡯ࠢࡒࡖࠥࡺࡲࡢࡰࡶࡰࡦࡺࡩࡰࡰࠣࡪࡦ࡯࡬ࡦࡦࠥ࠘"))
    if l11l1l_opy_==2.5:
        l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠣࡕࡨࡧࡴࡴࡤࡢࡴࡼࠤࡴࡨࡪࡦࡥࡷ࡭ࡻ࡫࠺ࠡࡒࡄࡗࡘࡋࡄࠣ࠙"))
    else: l1_opy_.l11lll_opy_().info(l1l1ll_opy_ (u"ࠤࡖࡩࡨࡵ࡮ࡥࡣࡵࡽࠥࡵࡢ࡫ࡧࡦࡸ࡮ࡼࡥ࠻ࠢࡉࡅࡎࡒࡅࡅࠤࠚ"))
    l1l_opy_ += l1llll_opy_
    l1l_opy_ += l11l1l_opy_
    l1_opy_.l11lll_opy_().info(l11ll1_opy_ (u"ࠥࡋࡷࡧࡤࡦ࠼ࠣࡿ࡬ࡸࡡࡥࡧࢀࠦࠛ"))
    l1_opy_.l1l1l1l_opy_.l11l1_opy_.join()
    rclpy.shutdown()
if __name__ == l1l1ll_opy_ (u"ࠫࡤࡥ࡭ࡢ࡫ࡱࡣࡤ࠭ࠜ"):
    main()
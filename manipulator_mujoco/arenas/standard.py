from dm_control import mjcf

class StandardArena(object):
    #创建一个纹理、材质、并添加到plane上加到worldbody中，最后添加光源
    def __init__(self) -> None:
        """
        Initializes the StandardArena object by creating a new MJCF model and adding a checkerboard floor and lights.
        """
        self._mjcf_model = mjcf.RootElement()

        self._mjcf_model.option.timestep = 0.002
        self._mjcf_model.option.flag.warmstart = "enable"

        # TODO don't use checker floor in future
        chequered = self._mjcf_model.asset.add(
            "texture",
            type="2d",
            builtin="checker",
            width=300,
            height=300,
            rgb1=[0.2, 0.3, 0.4],
            rgb2=[0.3, 0.4, 0.5],
        )
        grid = self._mjcf_model.asset.add(
            "material",
            name="grid",
            texture=chequered,
            texrepeat=[5, 5],
            reflectance=0.2,
        )
        self._mjcf_model.worldbody.add("geom", type="plane", size=[2, 2, 0.1], material=grid)
        for x in [-2, 2]:
            # TODO randomize lighting?
            self._mjcf_model.worldbody.add("light", pos=[x, -1, 3], dir=[-x, 1, -2])#添加光源
    #添加机械臂模型到仿真模型中
    def attach(self, child, pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        """
        Attaches a child element to the MJCF model at a specified position and orientation.

        Args:
            child: The child element to attach.
            pos: The position of the child element.
            quat: The orientation of the child element.

        Returns:
            The frame of the attached child element.
        """
        frame = self._mjcf_model.attach(child)
        frame.pos = pos
        frame.quat = quat
        return frame
    #添加方块到模型中
    def attach_free(self, child,  pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        """
        Attaches a child element to the MJCF model with a free joint.

        Args:
            child: The child element to attach.

        Returns:
            The frame of the attached child element.
        """
        frame = self.attach(child)
        frame.add('freejoint')
        frame.pos = pos
        frame.quat = quat
        return frame
    
    @property
    def mjcf_model(self) -> mjcf.RootElement:
        """
        Returns the MJCF model for the StandardArena object.

        Returns:
            The MJCF model.
        """
        return self._mjcf_model
class ColorRecognition:
    """
    Detects which patch the robot is standing on, from the
    two ground reflectance sensors.

    A candidate patch must be observed on 3 consecutive reads before 
    it replaces `current_patch`, to filter out sensor noise during transitions.
    """
    PATCHES = [
        {"index": 0, "average": 42, "eps": 14, "name": "black"},  
        {"index": 1, "average": 690, "eps": 27, "name": "white"},  
        {"index": 2, "average": 110, "eps": 24, "name": "brown"}   
    ]

    def __init__(self):
        self.current_patch = -1
        self.current_color = "floor"
        self.candidate = -1
        self.count = 0

    def find_color(self, ground):
        """
        Patch detection from a ground sensor reading.

        ground: [sensor_left, sensor_right] reflectance values.
        Returns: (patch_index, patch_name), or (-1, "floor") if no patch matches 
        """
        reflected = ground[0] + ground[1]
        for i in range(3):
            if reflected < 2*(self.PATCHES[i]["average"] + 2*self.PATCHES[i]["eps"]) and reflected > 2*(self.PATCHES[i]["average"] - 2*self.PATCHES[i]["eps"]):
                return i,self.PATCHES[i]["name"]
        return -1,"floor"

    def new_find_color(self, ground):
        """
        Patch detection from a ground sensor reading. Adapted to the change in the reading by the end of the experiment. 

        ground: [sensor_left, sensor_right] reflectance values.
        Returns: (patch_index, patch_name), or (-1, "floor") if no patch matches 
        """
        reflected = ground[0] + ground[1]
        if reflected < 300:
            return 0, "black"
        elif reflected < 550:
            return 2, "brown"
        elif reflected > 1840:
            return 1, "white"
        else:
            return -1, "floor"

    def filtered_color(self,ground):
        """
        Only updates current_patch/current_color once the same candidate has 
        been seen 3 times in a row.
        Returns: (current_patch, current_color).
        """
        patch, name = self.find_color(ground)
        if patch == self.candidate:
            self.count += 1
        else:
            self.candidate = patch
            self.count = 1
        if self.count >= 3:
            self.current_patch = patch
            self.current_color = name
        return self.current_patch, self.current_color

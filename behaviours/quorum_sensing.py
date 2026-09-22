class QuorumSensing:
    """ 
    Implements the quorum sensing algorithm which relies on majority voting.
    The robots might get their first opinion from the first patch they 
    encounter. Then they may change their opinion if more than 60% of 
    their neighbours share the same opinion.
    """

    QUORUM_THRESHOLD = 0.6

    def __init__(
            self
        ):
            self.patch = -1
            self.opinion = -1

    def tick(self,patch, neighbours):
        """
        Run one quorum-sensing tick.

        patch: index of the floor patch currently detected (-1 if none).
        neighbours: dict {id: {"id","tick","opinion"}} of neighbours.

        Returns: the robot's current opinion.
        """
        self.patch = patch
        if patch != -1 :
            if self.opinion == -1:
                self.opinion = patch

        # Robots with no opinion yet (never visited a patch) ignore
        # neighbours -- an opinion can only start from a patch.
        if self.opinion == -1:
            neighbours = {}

        opinions = [0]*3
        for n in neighbours.values():
            if n["opinion"] != -1:
                opinions[n["opinion"]] += 1
        max_neighbours = 0
        new_opinion = None
        for i in range(3):
            if opinions[i] > max_neighbours : 
                max_neighbours = opinions[i]
                new_opinion = i
        if len(neighbours) > 0:
            quorum = max_neighbours/len(neighbours)
        else : 
            quorum = 0
        if quorum > self.QUORUM_THRESHOLD and new_opinion != self.opinion :
            self.opinion = new_opinion
        return self.opinion
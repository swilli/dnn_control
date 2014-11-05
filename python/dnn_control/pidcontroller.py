import controller

class PIDController(controller.Controller):


	# Overrides Controller::get_action
	def get_action(self, state):
		return [0,0,0]
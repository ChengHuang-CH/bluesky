import numpy as np
from bluesky import stack, traf
from bluesky.tools.geo import qdrdist


# Radius (in "Latitudes")
RADIUS = 0.4
# Radius (in nautical miles)
_, RADIUS_NM = qdrdist(-RADIUS, 0, 0, 0)
# Separation (in nm)
SEP = 4.0
# Max distance (in nm)
MAX_DIST = RADIUS_NM*4
# Unit exchange
m2nm = 1.0/1852.0
# Command list (change direction)
COMMAND = [-5.0, 0.0, 5.0]
# Parameters used in simulation
MAX_EPI = 50
C_DF = 1.0
C_EX = 1.0
MAX_FIT = 1.0e2
# GA parameters
MAX_GEN = 1000
POP_SIZE = 200
ELITE = 5
CROSS = 0.5
MUTATION = 0.05


class NeuralNetwork:
	"""
	Create a simple Neural Network class manually
	"""
	def __init__(self, ni, nh, no):
		self.ni = ni
		self.nh = nh
		self.no = no
		self.w1 = np.random.uniform(-1, 1, (self.ni, self.nh))  # [ni, nh]
		self.b1 = np.random.uniform(-1, 1, (1, self.nh))  # [1, nh]
		self.w2 = np.random.uniform(-1, 1, (self.nh, self.no))  # [nh, no]
		self.b2 = np.random.uniform(-1, 1, (1, self.no))   # [1, no]

	def sigmoid(self, s):
		return 1 / (1 + np.exp(-s))

	def sigmoid_der(self, s):
		return self.sigmoid(s)*(1-self.sigmoid(s))

	def feedforward(self, X):  # X [m, ni]
		self.z1 = np.dot(X, self.w1) + self.b1  # [m, nh]
		self.p1 = self.sigmoid(self.z1)	# [m, nh]
		self.z2 = np.dot(self.p1, self.w2) + self.b2  # [m, no]
		self.p2 = self.sigmoid(self.z2)  # [m, no]
		return self.p2


# Initialise nn population and fitness values
NN_POP = [NeuralNetwork(8, 15, 3) for _ in range(POP_SIZE)]
NN_FIT = [0.0 for _ in range(POP_SIZE)]
EPISODE = 0
COUNT = 0
GEN = 0


def init_plugin():
	stack.stack(f'CIRCLE SIMAREA 0 0 {RADIUS_NM}')  # Draw a circle area
	reset_episode()
	# Configuration parameters
	config = {
		'plugin_name':	 'NN_2AC',
		'plugin_type':	 'sim',
		'update_interval': 10,
		'update':		  update,
		'preupdate':	  preupdate,
		}
	stackfunctions = {}
	return config, stackfunctions


# The update function is called after traffic is updated. Use this if you
# want to do things as a result of what happens in traffic. If you need to
# something before traffic is updated please use preupdate.
def update():
	global EPISODE, COUNT, GEN, NN_POP, NN_FIT

	# Iterate over all episodes for each nn
	if EPISODE == MAX_EPI:
		print(f'{MAX_EPI} episodes done for # {COUNT} NN')
		NN_FIT[COUNT] = NN_FIT[COUNT] / MAX_EPI  # average fit for all episodes
		EPISODE = 0  # start episode over again
		COUNT += 1

	# Iterate over all nn population
	if COUNT == POP_SIZE:
		print(f'Generation {GEN} done')
		print(f'Best fitness: {min(NN_FIT)}')
		print(f'Best NN: # {np.argmin(NN_FIT)}')
		run_ga(ELITE, CROSS, MUTATION)  # update nn
		NN_FIT = [0.0 for _ in range(POP_SIZE)]  # reset fitness
		COUNT = 0  # start count over again
		GEN += 1  # move to next generation

	# Iterate over all GA generations
	if GEN == MAX_GEN:
		print(f'Training done!')
		stack.stack('RESET')

	# Separation lower than safety one
	features = cal_input()

	if features[3] < SEP:
		NN_FIT[COUNT] += MAX_FIT
		reset_episode()
		EPISODE += 1  # move to next episode
	else:
		command(features)

	# Self aircraft leaves out of the circle area
	f = traf.id2idx('SSSS')
	_, dist = qdrdist(traf.lat[f], traf.lon[f], 0.0, 0.0)
	dist_flown = traf.distflown[f]*m2nm
	if dist > RADIUS_NM or dist_flown > MAX_DIST:
		NN_FIT[COUNT] += evaluate(features)
		reset_episode()
		EPISODE += 1  # move to next episode


# The preupdate function is called before traffic is updated. Use this
# function to provide settings that need to be used by traffic in the current
# timestep. Examples are ASAS, which can give autopilot commands to resolve
# a conflict.
def preupdate():
	pass


def create_self_ac():
	"""
	Create aircraft going from South to North
	"""
	pos_lat = -RADIUS
	pos_lon = 0
	hdg = 0
	stack.stack(f'CRE SSSS B737 {pos_lat} {pos_lon} {hdg} FL300 330')


def create_intruder_ac():
	"""
	Create aircraft coming inwards from a random point on circumference
	"""
	# np.random.seed(0)
	hdg = np.random.uniform(0, 360.0)
	hdg_r = np.deg2rad(hdg)
	pos_lat = -1 * RADIUS * np.cos(hdg_r)
	pos_lon = -1 * RADIUS * np.sin(hdg_r)
	stack.stack(f'CRE IIII B737 {pos_lat} {pos_lon} {hdg} FL300 330')


def delete_ac():
	stack.stack(f"DEL SSSS")
	stack.stack(f"DEL IIII")


def reset_episode():
	"""
	Reset aircraft after each simulation episode
	"""
	delete_ac()
	create_self_ac()
	create_intruder_ac()


def cal_input():
	"""
	Compute NN's input based on traffic objects
	"""
	f, fp = traf.id2idx('SSSS'), traf.id2idx('IIII')
	f_hdg, fp_hdg = traf.hdg[f], traf.hdg[fp]
	f_spd, fp_spd = traf.gs[f], traf.gs[fp]
	f_lat, fp_lat = traf.lat[f], traf.lat[fp]
	f_lon, fp_lon = traf.lon[f], traf.lon[fp]
	fex_lat, fex_lon = RADIUS, 0

	ffp_ang, ffp_dis = qdrdist(f_lat, f_lon, fp_lat, fp_lon)

	f_vec_spd = f_spd*np.cos((f_hdg - ffp_ang)/180 * np.pi)
	fp_vec_spd = fp_spd*np.cos((fp_hdg - ffp_ang)/180 * np.pi) * (-1)
	ffp_spd = f_vec_spd + fp_vec_spd

	fex_ang, fex_dis = qdrdist(f_lat, f_lon, fex_lat, fex_lon)
	fex_spd = f_spd*np.cos((f_hdg - fex_ang)/180 * np.pi)

	features = np.array(
		(f_hdg, f_spd, ffp_ang, ffp_dis, ffp_spd, fex_ang, fex_dis, fex_spd))
	return features


def command(features):
	"""
	Feed forward neural network
	"""
	cur_nn = NN_POP[COUNT]
	f_hdg = features[0]
	f_comd = f_hdg + COMMAND[cur_nn.feedforward(features).argmax()]
	stack.stack(f'HDG SSSS {f_comd}')


def evaluate(features):
	"""
	Calculate for fitness (dist flown + dist between actual and planned exit point)
	"""
	f = traf.id2idx('SSSS')
	dist_flown = traf.distflown[f]*m2nm
	fex_dis = features[6]
	return dist_flown*C_DF + fex_dis*C_EX


def m2v():
	"""
	Transform nn weights matrix to a vector (1d numpy array)
	"""
	nn_pop_vector = []
	for nn in NN_POP:
		w_vector = np.concatenate((nn.w1.flatten(), nn.b1.flatten(),
			nn.w2.flatten(), nn.b2.flatten()))
		nn_pop_vector.append(w_vector)
	return nn_pop_vector


def v2m(nn_pop_vector):
	"""
	Transform nn vector to weights matrix
	"""
	global NN_POP
	for i in range(len(NN_POP)):
		nn = NN_POP[i]
		n_w1, n_b1, n_w2, n_b2 = nn.ni*nn.nh, nn.nh, nn.nh*nn.no, nn.no
		update_w1 = np.array(nn_pop_vector[i][:n_w1])
		update_b1 = np.array(nn_pop_vector[i][n_w1:n_w1+n_b1])
		update_w2 = np.array(nn_pop_vector[i][n_w1+n_b1:n_w1+n_b1+n_w2])
		update_b2 = np.array(nn_pop_vector[i][n_w1+n_b1+n_w2:])

		NN_POP[i].w1 = update_w1.reshape((nn.ni, nn.nh))
		NN_POP[i].b1 = update_b1.reshape((1, nn.nh))
		NN_POP[i].w2 = update_w2.reshape((nn.nh, nn.no))
		NN_POP[i].b2 = update_b2.reshape((1, nn.no))


def crossover(nn1, nn2, p_cross):
	new_nn = nn1.copy()
	for i in range(len(new_nn)):
		rand = np.random.uniform()
		if rand > p_cross:
			new_nn[i] = nn2[i]
	return new_nn


def mutation(nn, p_mutation):
	new_nn = nn.copy()
	for i in range(len(new_nn)):
		rand = np.random.uniform()
		if rand < p_mutation:
			new_nn[i] = np.random.uniform(-1, 1)
	return new_nn


def run_ga(elite_size, p_cross, p_mutation):
	"""
	Run genetic algorithm
	"""
	nn_pop_vector = m2v()  # transform NN_POP to vector format
	ranked_nn_fit = list(np.argsort(NN_FIT))
	elite_set = [nn_pop_vector[i] for i in ranked_nn_fit[:elite_size]]
	rev_nn_fit = [1/i for i in NN_FIT]  # the lower fitness the higher chance
	select_probs = np.array(rev_nn_fit) / np.sum(rev_nn_fit)
	child_set = [crossover(
		nn_pop_vector[np.random.choice(range(POP_SIZE), p=select_probs)],
		nn_pop_vector[np.random.choice(range(POP_SIZE), p=select_probs)], p_cross)
		for _ in range(POP_SIZE - elite_size)]
	mutated_set = [mutation(nn, p_mutation) for nn in child_set]
	nn_pop_vector = elite_set
	nn_pop_vector += mutated_set
	v2m(nn_pop_vector)  # update NN_POP

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 *  Javier Bali�as Santos <javier@arc-robots.org>
 */

#include <stdio.h>
#include <stdint.h>

#include <aversive.h>
#include <aversive\pgmspace.h>
#include <aversive\wait.h>
#include <aversive\error.h>

#include <uart.h>
#include <i2c.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_mc.h>
#include <encoders_dspic.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include "main.h"
#include "beacon.h"
#include "beacon_calib.h"

/* debug macros */
#define BEACON_DEBUG(args...) DEBUG(E_USER_BEACON, args)
#define BEACON_NOTICE(args...) NOTICE(E_USER_BEACON, args)
#define BEACON_ERROR(args...) ERROR(E_USER_BEACON, args)

/* 180 deg. sensor adjusts - Opponent detection:
	Angle: at 10 cm of a vertical white paper, external ring (low point tangent) at 102 mm high.
	Power: with the angle adjusted, no detection at 39 cm with a vertical white paper.
*/

/* 0 deg. Sensor adjusts -  Second robot detection:
	Angle: at 10 cm of a vertical white paper, external ring (low point tangent) at 95.5 mm high.
	Power: with the angle adjusted, no detection at 20 cm with a vertical white paper.

	NOTE: beacon needs to add a black tape of 1cm at the top of beacon in order to the second
			robot's beacon don't be detected by 180deg sensor (opponents measurements).
*/


/* SENSOR 180 ***************************************************************************/

/* size measure params */
#define S180_MEASURE_MAX	5400
#define S180_MEASURE_MIN 	920

/* get distance params */
#define S180_X_EVAL_DELTA	8
#define S180_X_EVAL_MIN		920
#define S180_ARRAY_SIZE		561

uint8_t measure2distance_sensor_180 [] = {
23	,
23	,
24	,
24	,
25	,
25	,
26	,
27	,
27	,
28	,
28	,
29	,
29	,
30	,
30	,
31	,
32	,
32	,
33	,
33	,
34	,
34	,
35	,
36	,
36	,
37	,
37	,
38	,
38	,
39	,
40	,
40	,
41	,
41	,
42	,
42	,
43	,
44	,
44	,
45	,
45	,
46	,
46	,
47	,
48	,
48	,
49	,
49	,
50	,
50	,
51	,
51	,
52	,
53	,
53	,
54	,
54	,
55	,
55	,
56	,
56	,
57	,
57	,
58	,
58	,
59	,
60	,
60	,
61	,
61	,
62	,
62	,
63	,
63	,
64	,
64	,
65	,
65	,
66	,
66	,
67	,
67	,
68	,
68	,
69	,
69	,
69	,
70	,
70	,
71	,
71	,
72	,
72	,
73	,
73	,
74	,
74	,
74	,
75	,
75	,
76	,
76	,
77	,
77	,
77	,
78	,
78	,
79	,
79	,
79	,
80	,
80	,
81	,
81	,
81	,
82	,
82	,
83	,
83	,
83	,
84	,
84	,
84	,
85	,
85	,
86	,
86	,
86	,
87	,
87	,
87	,
88	,
88	,
88	,
89	,
89	,
89	,
90	,
90	,
90	,
91	,
91	,
91	,
92	,
92	,
92	,
93	,
93	,
93	,
93	,
94	,
94	,
94	,
95	,
95	,
95	,
96	,
96	,
96	,
96	,
97	,
97	,
97	,
98	,
98	,
98	,
98	,
99	,
99	,
99	,
99	,
100	,
100	,
100	,
101	,
101	,
101	,
101	,
102	,
102	,
102	,
102	,
103	,
103	,
103	,
103	,
104	,
104	,
104	,
104	,
105	,
105	,
105	,
105	,
106	,
106	,
106	,
106	,
107	,
107	,
107	,
107	,
108	,
108	,
108	,
108	,
109	,
109	,
109	,
109	,
110	,
110	,
110	,
110	,
111	,
111	,
111	,
111	,
112	,
112	,
112	,
112	,
113	,
113	,
113	,
113	,
114	,
114	,
114	,
114	,
115	,
115	,
115	,
115	,
116	,
116	,
116	,
116	,
117	,
117	,
117	,
117	,
118	,
118	,
118	,
119	,
119	,
119	,
119	,
120	,
120	,
120	,
120	,
121	,
121	,
121	,
122	,
122	,
122	,
122	,
123	,
123	,
123	,
124	,
124	,
124	,
124	,
125	,
125	,
125	,
126	,
126	,
126	,
127	,
127	,
127	,
127	,
128	,
128	,
128	,
129	,
129	,
129	,
130	,
130	,
130	,
131	,
131	,
131	,
132	,
132	,
132	,
133	,
133	,
133	,
134	,
134	,
134	,
135	,
135	,
135	,
136	,
136	,
136	,
137	,
137	,
137	,
138	,
138	,
138	,
139	,
139	,
139	,
140	,
140	,
141	,
141	,
141	,
142	,
142	,
142	,
143	,
143	,
143	,
144	,
144	,
145	,
145	,
145	,
146	,
146	,
147	,
147	,
147	,
148	,
148	,
148	,
149	,
149	,
150	,
150	,
150	,
151	,
151	,
152	,
152	,
152	,
153	,
153	,
154	,
154	,
154	,
155	,
155	,
156	,
156	,
156	,
157	,
157	,
158	,
158	,
158	,
159	,
159	,
160	,
160	,
161	,
161	,
161	,
162	,
162	,
163	,
163	,
163	,
164	,
164	,
165	,
165	,
166	,
166	,
166	,
167	,
167	,
168	,
168	,
168	,
169	,
169	,
170	,
170	,
170	,
171	,
171	,
172	,
172	,
173	,
173	,
173	,
174	,
174	,
175	,
175	,
175	,
176	,
176	,
177	,
177	,
177	,
178	,
178	,
179	,
179	,
179	,
180	,
180	,
181	,
181	,
181	,
182	,
182	,
183	,
183	,
183	,
184	,
184	,
184	,
185	,
185	,
186	,
186	,
186	,
187	,
187	,
187	,
188	,
188	,
189	,
189	,
189	,
190	,
190	,
190	,
191	,
191	,
191	,
192	,
192	,
192	,
193	,
193	,
194	,
194	,
194	,
195	,
195	,
195	,
196	,
196	,
196	,
197	,
197	,
197	,
197	,
198	,
198	,
198	,
199	,
199	,
199	,
200	,
200	,
200	,
201	,
201	,
201	,
201	,
202	,
202	,
202	,
203	,
203	,
203	,
203	,
204	,
204	,
204	,
204	,
205	,
205	,
205	,
205	,
206	,
206	,
206	,
207	,
207	,
207	,
207	,
207	,
208	,
208	,
208	,
208	,
209	,
209	,
209	,
209	,
210	,
210	,
210	,
210	,
211	,
211	,
211	,
211	,
211	,
212	,
212	,
212	,
212	,
212	,
213	,
213	,
213	,
213	,
214	,
214	,
214	,
214	,
214	,
215	,
215	,
215	,
215	,
215	,
216	,
216	,
216	,
216	,
216	,
217	,
217	,
217	,
217	,
217	,
218	,
218	,
218	,
218	,
219	,
219	,
219	,
219	,
219	,
220	,
220	,
220	,
220	,
221	,
221	,
221 };

/* SENSOR 0 ***************************************************************************/

/* size measure params */
#define S0_MEASURE_MAX	4500
#define S0_MEASURE_MIN 	820

/* get distance params */
#define S0_X_EVAL_DELTA		8
#define S0_X_EVAL_MIN		820
#define S0_ARRAY_SIZE		461

uint16_t measure2distance_sensor_0 [] = {
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
30	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
31	,
32	,
32	,
32	,
32	,
32	,
32	,
32	,
32	,
32	,
33	,
33	,
33	,
33	,
33	,
33	,
33	,
33	,
34	,
34	,
34	,
34	,
34	,
34	,
34	,
35	,
35	,
35	,
35	,
35	,
35	,
36	,
36	,
36	,
36	,
36	,
37	,
37	,
37	,
37	,
37	,
38	,
38	,
38	,
38	,
38	,
39	,
39	,
39	,
39	,
40	,
40	,
40	,
40	,
41	,
41	,
41	,
41	,
42	,
42	,
42	,
42	,
43	,
43	,
43	,
44	,
44	,
44	,
44	,
45	,
45	,
45	,
46	,
46	,
46	,
47	,
47	,
47	,
47	,
48	,
48	,
48	,
49	,
49	,
49	,
50	,
50	,
50	,
51	,
51	,
51	,
52	,
52	,
53	,
53	,
53	,
54	,
54	,
54	,
55	,
55	,
56	,
56	,
56	,
57	,
57	,
57	,
58	,
58	,
59	,
59	,
59	,
60	,
60	,
61	,
61	,
61	,
62	,
62	,
63	,
63	,
64	,
64	,
64	,
65	,
65	,
66	,
66	,
67	,
67	,
67	,
68	,
68	,
69	,
69	,
70	,
70	,
71	,
71	,
72	,
72	,
72	,
73	,
73	,
74	,
74	,
75	,
75	,
76	,
76	,
77	,
77	,
78	,
78	,
79	,
79	,
80	,
80	,
80	,
81	,
81	,
82	,
82	,
83	,
83	,
84	,
84	,
85	,
85	,
86	,
86	,
87	,
87	,
88	,
88	,
89	,
89	,
90	,
90	,
91	,
91	,
92	,
92	,
93	,
93	,
94	,
94	,
95	,
95	,
96	,
96	,
97	,
97	,
98	,
98	,
99	,
99	,
100	,
100	,
101	,
101	,
102	,
102	,
103	,
103	,
104	,
104	,
105	,
105	,
106	,
106	,
107	,
107	,
108	,
108	,
109	,
109	,
110	,
110	,
111	,
111	,
112	,
112	,
113	,
113	,
114	,
114	,
115	,
115	,
116	,
116	,
117	,
117	,
118	,
118	,
119	,
119	,
120	,
120	,
121	,
121	,
122	,
122	,
123	,
123	,
124	,
124	,
125	,
125	,
125	,
126	,
126	,
127	,
127	,
128	,
128	,
129	,
129	,
130	,
130	,
131	,
131	,
132	,
132	,
132	,
133	,
133	,
134	,
134	,
135	,
135	,
136	,
136	,
137	,
137	,
137	,
138	,
138	,
139	,
139	,
140	,
140	,
141	,
141	,
141	,
142	,
142	,
143	,
143	,
144	,
144	,
144	,
145	,
145	,
146	,
146	,
147	,
147	,
147	,
148	,
148	,
149	,
149	,
149	,
150	,
150	,
151	,
151	,
152	,
152	,
152	,
153	,
153	,
154	,
154	,
154	,
155	,
155	,
156	,
156	,
157	,
157	,
157	,
158	,
158	,
159	,
159	,
159	,
160	,
160	,
161	,
161	,
162	,
162	,
162	,
163	,
163	,
164	,
164	,
165	,
165	,
165	,
166	,
166	,
167	,
167	,
168	,
168	,
168	,
169	,
169	,
170	,
170	,
171	,
171	,
172	,
172	,
173	,
173	,
173	,
174	,
174	,
175	,
175	,
176	,
176	,
177	,
177	,
178	,
178	,
179	,
179	,
180	,
180	,
181	,
181	,
182	,
183	,
183	,
184	,
184	,
185	,
185	,
186	,
187	,
187	,
188	,
188	,
189	,
190	,
190	,
191	,
191	,
192	,
193	,
193	,
194	,
195	,
196	,
196	,
197	,
198	,
198	,
199	,
200	,
201	,
201	,
202	,
203	,
204	,
205	,
205	,
206	,
207	,
208	,
209	,
210	,
211	,
211	,
212	,
213	,
214	,
215	,
216	,
217	,
218	,
219	,
220	,
221	,
222	,
223	,
224	,
226	,
227 };

/* get distance (cm) from relative size */
uint16_t get_dist_array(uint8_t sensor, int32_t size, int32_t period)
{
	uint32_t measure;
	uint16_t index;

	/* calculate measure */
	measure = (uint32_t)((size*100000)/period);

	if(sensor == IR_SENSOR_180_DEG) 
	{
		/* saturate to working range */
		if(measure > S180_MEASURE_MAX)
			measure = S180_MEASURE_MAX;
		if(measure < S180_MEASURE_MIN)
			measure = S180_MEASURE_MIN;
	
		/* calculate array index */
		index = (S180_ARRAY_SIZE-1) - ((measure-S180_X_EVAL_MIN)/S180_X_EVAL_DELTA);	
	
		//BEACON_DEBUG("measure = %.4ld / index = %.3d / dist = %.3d cm",
		//		       measure, index, measure2distance_sensor_180[index]);
	
		/* return distance from array in mm */
		return (measure2distance_sensor_180[index]*10);
	}
	else 
	{
		/* HACK: because sensor intesity power was changed before calibration */
		//measure += 1008;

		/* saturate to working range */
		if(measure > S0_MEASURE_MAX)
			measure = S0_MEASURE_MAX;
		if(measure < S0_MEASURE_MIN)
			measure = S0_MEASURE_MIN;
	
		/* calculate array index */
		index = (S0_ARRAY_SIZE-1) - ((measure-S0_X_EVAL_MIN)/S0_X_EVAL_DELTA);	
	
		//BEACON_DEBUG("measure = %.4ld / index = %.3d / dist = %.3d cm",
		//		       measure, index, measure2distance_sensor_0[index]);
	
		/* return distance from array in mm */
		return (measure2distance_sensor_0[index]*10);
	}

}

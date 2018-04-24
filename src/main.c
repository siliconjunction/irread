#include "mgos.h"
#include "mgos_http_server.h"


#define BUF_SIZE 1000
#define SMALL_DIFF 60 // us
#define BIG_GAP 10000 // us
#define MIN_SAMPLES 10

#define STATE_IDLE		0	// magenta heartbeat
#define STATE_LEARN		1	// cyan
#define STATE_RECV		2	// red
#define STATE_LEARNT	3	// cyan
#define STATE_SEND		4	// green


#define CODING_PULSE	'M'
#define CODING_SPACE	'S'
#define CODING_PHASE	'P'
#define CODING_QUAD		'Q'

#define MG_MALLOC	malloc
#define MG_FREE		free

#define LED_PIN		2

#define INPUT_PIN 	5
#define LED_RGB		14

#define IR_OUT_PIN		4
#define PIN_ON			1
#define PIN_OFF			0


struct ir_signal {
	// data to be sent. not part of the protocol
	long data;
	// no. of times to repeat signal, not part of the protocol
	int repCount;

	// Protocol parameters

	// mark is presence of carrier signal
	// space is lack of carrier signal

	// coding scheme
	// M - pulse (or mark) encoding
	// S - space encoding
	// P - phase encoding
	// B - both mark and space (used in some rf protocols)
	// Q - quad encoding (used by foxtel remotes)
	char coding;
	// no. of bits of data
	int dataBits;
	// duration of header mark and space
	int headM, headS;
	// set of timings
	// interpretation depends on coding scheme
	int timings[5];
	// long pause, stop mark and space used in some remotes
	int pauseS;
	int stopM, stopS;
	// end mark used in space encoding and pause/stop signals
	int endM;
	// gap between subsequent signal trains
	int gap;
	// type of repeat in case of long press
	// R - repeat same cose
	// E - repeat stop/pause only
	// F- weird foxtel repeat
	char repeat;
	// non-standard gap used in foxtel
	int foxtelGap;
	// mask to change a bit in data in repeats
	long foxtelMask;
};


static struct ir_signal *getIRSignalFromURL(struct http_message *hm);
static int getInt(const char *p, int len, int * const inx, const int skip);
static long getLong(const char *p, int len, int * const inx, int skip);
static long getLongHex(const char *p, int len, int * const inx);
static void sendIRData(struct ir_signal * irs);
static void sendSingleIR(struct ir_signal *irs);
static void irsend_carrier_40kHz(int us);
static void set_learn();
static void timeout_cb(void *arg);
static void writeRgbLed();
static void timer_cb(void *arg);


int buffer[BUF_SIZE];
int bufInx = 0;
volatile int state = STATE_IDLE;
uint32_t timer = 0;
uint8_t rgbData[3];


static void cgi_handler(struct mg_connection *c, int ev, void *p,
		void *user_data) {
	struct http_message *hm = (struct http_message *)p;
	LOG(LL_INFO, ("73: URI=%.*s", hm->uri.len, hm->uri.p));
	LOG(LL_INFO, ("74: QS=%.*s", hm->query_string.len, hm->query_string.p));
	if (ev != MG_EV_HTTP_REQUEST) {
		return;
	}
	if (strncmp("/cgi/Send", hm->uri.p, hm->uri.len) == 0) {
		LOG(LL_INFO, ("79: Received Send"));
		struct ir_signal *irs = getIRSignalFromURL(hm);
		if (irs == NULL) {
			return;
		}
		LOG(LL_INFO, ("84: Send data %ld using C=%c,%d;H=%d,%d;T=%d,%d,%d,%d,%d;P=%d;S=%d,%d;E=%d;G=%d",
			irs->data, irs->coding, irs->dataBits,irs->headM,irs->headS,
			irs->timings[0],irs->timings[1],irs->timings[2],irs->timings[3],irs->timings[4],
			irs->pauseS,irs->stopM,irs->stopS,irs->endM,irs->gap));
		LOG(LL_INFO, ("89: Repeat R=%c,%d,%d,%lx",irs->repeat,irs->repCount,irs->foxtelGap,irs->foxtelMask));
		sendIRData(irs);
		// if ((dir = (struct win32_dir *) MG_MALLOC(sizeof(*dir))) == NULL) {
		MG_FREE(irs);

		mg_send_response_line(c, 200,
				"Content-Type: text/plain\r\n");
		mg_printf(c, "OK\r\n");
	}
	else if (strncmp("/cgi/Learn", hm->uri.p, hm->uri.len) == 0) {
		LOG(LL_INFO, ("97: Received Learn"));
		if (state == STATE_IDLE) {
			set_learn();
			mg_send_response_line(c, 200,
					"Content-Type: text/html\r\n");
			mg_printf(c, "[-1]\r\n");
		}
		else {
			LOG(LL_INFO, ("105: Learn in progress? %d", state));
			mg_send_response_line(c, 200,
					"Content-Type: text/html\r\n");
			mg_printf(c, "[-1]\r\n");
		}
	}
	else if (strncmp("/cgi/Read", hm->uri.p, hm->uri.len) == 0) {
		if (state == STATE_LEARNT) {
			LOG(LL_INFO, ("113: Learn complete"));
			mg_send_response_line(c, 200,
					"Content-Type: text/html\r\n");
			mg_printf(c, "[");
			int i;
			for (i = 0; i < bufInx; i++) {
				mg_printf(c, "%d,", buffer[i]);
			}
			mg_printf(c, "0]");
			state = STATE_IDLE;
		}
		else {
			LOG(LL_INFO, ("125: Learn in progress? %d", state));
			mg_send_response_line(c, 200,
					"Content-Type: text/html\r\n");
			mg_printf(c, "[]\r\n");
		}
	}
	else {
		LOG(LL_INFO, ("132: Received unknown"));
		mg_send_response_line(c, 200,
				"Content-Type: text/html\r\n");
		mg_printf(c, "%s\r\n", "Huh?");
	}
	c->flags |= MG_F_SEND_AND_CLOSE;
	(void) user_data;
}

// s=C=S,12;H=2200,1100;T=550,1100,550,,;P=455;S=50,50;E=550;G=22000;&d=12
// s=C~M_33-H~5500_6600-T~880_9009_100__-P~455-S~55_45-E~766-G~22000-&d=12
static struct ir_signal *getIRSignalFromURL(struct http_message *hm) {
	//struct ir_signal *irs = (struct ir_signal *) MG_MALLOC(sizeof(struct ir_signal));
	struct ir_signal irs_;
	const char *p = hm->query_string.p;
	int len = hm->query_string.len;
	int inx = 0;

	while (inx < len) {
		LOG(LL_INFO, ("151: char %c at %d", p[inx], inx));
		if (p[inx] == 's') {
			// get coding scheme
			inx += 4;
			if (inx >= len) {
				return NULL;
			}
			// s=C=S,12;H=2200,1100;T=550,1100,550,,;P=0;S=0,0;E=550;G=22000;R=R,3&d=12
			//     ^
			irs_.coding = p[inx];
			irs_.dataBits = getInt(p, len, &inx, 2);
			//            ^
			irs_.headM = getInt(p, len, &inx, 3);
			irs_.headS = getInt(p, len, &inx, 1);
			LOG(LL_INFO, ("165: %d, char %c at %d", irs_.headS, p[inx], inx));
			irs_.timings[0] = getInt(p, len, &inx, 3);
			irs_.timings[1] = getInt(p, len, &inx, 1);
			irs_.timings[2] = getInt(p, len, &inx, 1);
			irs_.timings[3] = getInt(p, len, &inx, 1);
			irs_.timings[4] = getInt(p, len, &inx, 1);
			irs_.pauseS = getInt(p, len, &inx, 3);;
			irs_.stopM = getInt(p, len, &inx, 3);
			irs_.stopS = getInt(p, len, &inx, 1);
			irs_.endM = getInt(p, len, &inx, 3);
			irs_.gap = getInt(p, len, &inx, 3);
			LOG(LL_INFO, ("176: %d, char %c at %d", irs_.gap, p[inx], inx));
			inx++;
			if (p[inx] == 'R') {
				inx += 2;
				irs_.repeat = p[inx++];
				irs_.repCount = getInt(p, len, &inx, 1);
				LOG(LL_INFO, ("182: %d, char %c at %d", irs_.repCount, p[inx], inx));
				irs_.foxtelGap = getInt(p, len, &inx, 1);
				LOG(LL_INFO, ("184: %d, char %c at %d", irs_.foxtelGap, p[inx], inx));
				irs_.foxtelMask = getLong(p, len, &inx, 1);
				LOG(LL_INFO, ("186: %lx, char %c at %d", irs_.foxtelMask, p[inx], inx));
			}
		}
		else if (p[inx] == 'd') {
			LOG(LL_INFO, ("185: char %c at %d", p[inx], inx));
			irs_.data = getLong(p, len, &inx, 2);
		}
		else {
			return NULL;
		}

		inx++;
	}

	struct ir_signal *irs = (struct ir_signal *) MG_MALLOC(sizeof(struct ir_signal));
	memcpy(irs, &irs_, sizeof(struct ir_signal));
	return irs;
}

static void sendIRData(struct ir_signal *irs) {
	rgbData[0] = 20;
	rgbData[1] = 0;
	rgbData[2] = 0;
	writeRgbLed();

	if (irs->repeat == 'R') {
		int count = irs->repCount;
		if (count == 0) {
			count = 1;
		}
		for (int inx = 0; inx < count; inx++) {
			sendSingleIR(irs);
		}
	}
	else if (irs->repeat == 'F') {
		int count = irs->repCount;
		if (count == 0) {
			count = 1;
		}
		for (int inx = 0; inx < count; inx++) {
			sendSingleIR(irs);
		}
		irs->gap = irs->foxtelGap;
		sendSingleIR(irs);
		irs->data ^= irs->foxtelMask;
		sendSingleIR(irs);
	}

	rgbData[0] = 0;
	rgbData[1] = 0;
	rgbData[2] = 0;
	writeRgbLed();
}

static void sendSingleIR(struct ir_signal *irs) {
	int inx = 0;
	int data = irs->data;

	if (irs->headM > 0) {
		// header
		irsend_carrier_40kHz(irs->headM);
		mgos_usleep(irs->headS);
	}

	if (irs->coding == 'Q') {
		for (inx = 0; inx < irs->dataBits; inx += 2) {
			int bit = (data & 3);
			irsend_carrier_40kHz(irs->timings[0]);
			if (bit == 0) {
				mgos_usleep(irs->timings[1]);
			}
			else if (bit == 1) {
				mgos_usleep(irs->timings[2]);
			}
			else if (bit == 2) {
				mgos_usleep(irs->timings[3]);
			}
			else if (bit == 3) {
				mgos_usleep(irs->timings[4]);
			}
			data >>= 2;
		}
	}
	else {
		for (inx = 0; inx < irs->dataBits; inx++) {
			int bit = (data & 1);
			if (irs->coding == 'M') {
				// irs->timings M0, M1, S
				if (bit == 0) {
					irsend_carrier_40kHz(irs->timings[0]);
				}
				else {
					irsend_carrier_40kHz(irs->timings[1]);
				}
				mgos_usleep(irs->timings[2]);
			}
			else if (irs->coding == 'S') {
				// irs->timings M, S0, S1
				irsend_carrier_40kHz(irs->timings[0]);
				if (bit == 0) {
					mgos_usleep(irs->timings[1]);
				}
				else {
					mgos_usleep(irs->timings[2]);
				}
			}
			else if (irs->coding == 'P') {
				if (bit = 0) {
					mgos_usleep(irs->timings[0]);
					irsend_carrier_40kHz(irs->timings[0]);
				}
				else {
					irsend_carrier_40kHz(irs->timings[0]);
					mgos_usleep(irs->timings[0]);
				}
			}
			data >>= 1;
		}
	}
	if (irs->pauseS > 0) {
		// pause
		mgos_usleep(irs->pauseS);
	}
	if (irs->stopM > 0) {
		// stop
		irsend_carrier_40kHz(irs->stopM);
		mgos_usleep(irs->stopS);
	}
	if (irs->endM > 0) {
		// end
		irsend_carrier_40kHz(irs->endM);
	}
	if (irs->gap > 0) {
		mgos_usleep(irs->gap);
	}
}

static int getInt(const char *p, int len, int * const inx, int skip) {
	int ret = 0;
	*inx += skip;
	while (p[*inx] >= '0' && p[*inx] <= '9' && *inx < len) {
		ret *= 10;
		ret += p[*inx] - '0';
		(*inx)++;
	}
	return ret;
}

static long getLong(const char *p, int len, int * const inx, int skip) {
	long ret = 0;
	*inx += skip;
	LOG(LL_INFO, ("328: char %c at %d", p[*inx], *inx));
	if (p[*inx + 1] == 'x') {
		LOG(LL_INFO, ("330: found x"));
		return getLongHex(p, len, inx);
	}
	while (p[*inx] >= '0' && p[*inx] <= '9' && *inx < len) {
		ret *= 10;
		ret += p[*inx] - '0';
		(*inx)++;
	}
	return ret;
}

static long getLongHex(const char *p, int len, int * const inx) {
	long ret = 0;
	*inx += 2;
	while (p[*inx] >= '0' && p[*inx] <= 'f' && *inx < len) {
		LOG(LL_INFO, ("345: char %c at %d", p[*inx], *inx));
		ret *= 16;
		char c = p[*inx];
		if (c <= '9') {
			ret += p[*inx] - '0';
		}
		else if (c <= 'F') {
			ret += p[*inx] - '7';
		}
		else {
			ret += p[*inx] - 'W';
		}
		(*inx)++;
	}
	return ret;
}

static void irsend_carrier_40kHz(int us) {
	us = ((us + 12) / 25);
	// make signal of circa 40 kHz of circa 1/3 duty
	for (; us >= 0; --us) {
		mgos_gpio_write(IR_OUT_PIN, PIN_ON);
		mgos_usleep(8);
		mgos_gpio_write(IR_OUT_PIN, PIN_OFF);
		mgos_usleep(16);
	}
}

int timerCount = 0;
static void timer_cb(void *arg) {
	if (++timerCount >= 10) {
		timerCount = 0;
		//LOG(LL_INFO, ("Debug: state=%d,buf=%d,time=%ld", state, bufInx, (long)timer));
	}
	if (state == STATE_IDLE) {
		if (timerCount == 0) {
			rgbData[0] = 0;
			rgbData[1] = 20;
			rgbData[2] = 20;
		}
		else {
			rgbData[0] = 0;
			rgbData[1] = 0;
			rgbData[2] = 0;
		}
		writeRgbLed();
	}
	else if (state == STATE_RECV) {
		/*
		rgbData[0] = 4;
		rgbData[1] = 0;
		rgbData[2] = 4;
		writeRgbLed();
		*/
	}
	(void) arg;
}

static void set_learn() {
	LOG(LL_INFO, ("372: %s", "Enter Learn mode"));

	rgbData[0] = 4;
	rgbData[1] = 0;
	rgbData[2] = 4;
	writeRgbLed();

	state = STATE_LEARN;
	bufInx = 0;
	timer = 1000000 * mgos_uptime();
	mgos_gpio_enable_int(INPUT_PIN);

	mgos_set_timer(10000 /* ms */, false /* repeat */, timeout_cb, NULL);
}

static void timeout_cb(void *arg) {
	LOG(LL_INFO, ("388: %s", "Timeout reached"));
	mgos_gpio_disable_int(INPUT_PIN);
	state = STATE_LEARNT;
	int i;
	for (i = 0; i < bufInx; i++) {
		LOG(LL_INFO, ("393: %d. %d", i, buffer[i]));
	}
	(void) arg;
}

static void irrecv_handler(int pin, void *arg) {
	if (pin != INPUT_PIN) {
		return;
	}
	mgos_gpio_write(mgos_sys_config_get_pins_led(), mgos_gpio_read(pin));

	if (state == STATE_LEARN) {
		timer = 1000000 * mgos_uptime();
		state = STATE_RECV;
	}
	else if (state == STATE_RECV) {
		uint32_t now = 1000000 * mgos_uptime();
		buffer[bufInx] = now - timer;
		timer = now;
		if (mgos_gpio_read(INPUT_PIN) == 0) {
			buffer[bufInx] = -buffer[bufInx];
		}
		if (++bufInx >= BUF_SIZE) {
			state = STATE_LEARNT;
		}
		/*
		rgbData[0] = 0;
		rgbData[1] = 100;
		rgbData[2] = 0;
		writeRgbLed();
		*/
	}

	(void) arg;
}

static void writeRgbLed() {
	mgos_gpio_write(LED_RGB, 0);
	mgos_usleep(60);
	mgos_bitbang_write_bits(LED_RGB, MGOS_DELAY_100NSEC, 3, 8, 7, 6,
		rgbData, sizeof(rgbData));
	mgos_gpio_write(LED_RGB, 0);
	mgos_usleep(60);
	mgos_gpio_write(LED_RGB, 1);
}

enum mgos_app_init_result mgos_app_init(void) {
	/* Blink built-in LED every second */
	mgos_gpio_set_mode(mgos_sys_config_get_pins_led(), MGOS_GPIO_MODE_OUTPUT);
	//mgos_set_timer(1000, MGOS_TIMER_REPEAT, led_timer_cb, NULL);

	mgos_gpio_set_mode(INPUT_PIN, MGOS_GPIO_MODE_INPUT);
	mgos_gpio_set_pull(INPUT_PIN, MGOS_GPIO_PULL_UP);
	mgos_gpio_set_int_handler_isr(INPUT_PIN, MGOS_GPIO_INT_EDGE_ANY, irrecv_handler, NULL);
	//(void)irrecv_handler;

    mgos_gpio_set_mode(LED_RGB, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(LED_RGB, 0);

	mgos_set_timer(200 /* ms */, true /* repeat */, timer_cb, NULL);
	//(void)timer_cb;

	LOG(LL_INFO, ("455: %s", "Entered main"));

	mgos_gpio_set_mode(IR_OUT_PIN, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_write(IR_OUT_PIN, PIN_OFF);
	mgos_register_http_endpoint("/cgi/", cgi_handler, NULL);
	LOG(LL_INFO, ("460: %s", "Registered endpoint"));

	return MGOS_APP_INIT_SUCCESS;
}

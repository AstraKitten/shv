// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define maxControllers 6

int emulate = 0;
int heartbeat = 0;
int delayMode = 0;
int fd[maxControllers];
int c = 0;
int startInterval = 4;
int usleepAmt = 0;
int paused = 1;
int primaryControlAllowed = 0;
int secondaryControlAllowed = 0;
int auxMode = 0;
int auxModeOsc = 0;
int numControllers = 0;
const char* controllerPath[maxControllers];
int activeControllers = 0;

const char* controllerMap = "dualshock";
int buttonX, buttonO, buttonTriangle, buttonSquare, buttonSelect, buttonRB, buttonRT;

int randomRange(int min, int max)
{
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

void handlePrimaryControl(int value)
{
    primaryControlAllowed = value;
    if (primaryControlAllowed) {
        printf("\nPrimary control enabled");
    } else {
        printf("\nPrimary control disabled");
    }
}

void handleSecondaryControl(int value)
{
    secondaryControlAllowed = value;
    if (secondaryControlAllowed) {
        printf("\nSecondary control enabled");
    } else {
        printf("\nSecondary control disabled");
    }
}

void handlePause()
{
    paused = !paused;
    printf("\nToggled");
}

void handleModeCycle()
{
    if (heartbeat) {
        usleepAmt = 25000 * startInterval;
        printf("\nSwitched to interval mode");
    } else {
        usleepAmt = 50000;
        printf("\nSwitched to heartbeat mode");
        if (auxMode > 1) {
            auxMode = 1;
            printf("\nAuxillary motor enabled: steady");
        }
    }
    heartbeat = !heartbeat;
    c = 0;
}

void handleDelayCycle()
{
    if (delayMode == 0) {
        delayMode = 1;
        printf("\nSet delay mode to random");
    } else if (delayMode > 0 && delayMode < 9) {
        delayMode += 1;
        printf("\nSet delay mode to stable interval %d", (delayMode - 1));
    } else if (delayMode >= 9) {
        delayMode = 0;
        printf("\nSet delay mode to none");
    }
}

void handleFrequencyUp()
{
    if (heartbeat) {
        if (c <= 1000) {
            c = (((c / 100) + 1) * 100);
        }
    } else {
        if (usleepAmt >= 50000) {
            usleepAmt -= 25000;
        }
        printf("\nAdjusted delay to %d", usleepAmt);
    }
}

void handleFrequencyDown()
{
    if (heartbeat) {
        if (c >= 100) {
            c = (((c / 100) - 1) * 100);
        }
    } else {
        if (usleepAmt <= 175000) {
            usleepAmt += 25000;
        }
        printf("\nAdjusted delay to %d", usleepAmt);
    }
}

void handleAux()
{
    if (auxMode == 0) {
        auxMode = 1;
        printf("\nAuxillary motor enabled: steady");
    } else if (auxMode > 0 && auxMode < 7 && !heartbeat) {
        auxMode++;
        printf("\nAuxillary motor enabled: alternating mode %d", (auxMode - 1));
    } else if (auxMode >= 7 || heartbeat) {
        auxMode = 0;
        printf("\nAuxillary motor disabled");
    }
}

void handleAuxOsc()
{
    auxModeOsc = !auxModeOsc;
    printf("\nAuxillary motor oscillating mode toggled");
}

void handleActiveControllerCycle()
{
    activeControllers += 1;
    if (activeControllers > numControllers) {
        activeControllers = 0;
    }
    if (activeControllers == 0) {
        printf("\nAll controllers activated");
    } else {
        printf("\nController %d activated", activeControllers);
    }
}

void* read_input(void*)
{
    while (1) {
        struct input_event event;

        ssize_t bytes_read = read(fd[0], &event, sizeof(struct input_event));
        if (bytes_read == -1) {
            perror("Failed to read controller inputs");
            close(fd[0]);
            exit(1);
            break;
        }
        if (bytes_read == 0) {
            continue;
        }

        // https://www.kernel.org/doc/html/latest/input/event-codes.html
        switch (event.type) {
        case EV_KEY:
            if (event.code == buttonX) { // X
                handlePrimaryControl(event.value);
            }

            if (event.code == buttonTriangle) { // Triangle
                handleSecondaryControl(event.value);
            }

            if (event.code == buttonSelect && event.value == 1) { // Select / Options
                handlePause();
            }

            if (primaryControlAllowed && !secondaryControlAllowed) {
                if (event.code == buttonO && event.value == 1) { // X + O
                    handleModeCycle();
                }

                if (event.code == buttonSquare && event.value == 1) { // X + Square
                    handleDelayCycle();
                }

                if (event.code == buttonRB && event.value == 1) { // X + RB
                    handleFrequencyUp();
                }

                if (event.code == buttonRT && event.value == 1) { // X + RT
                    if (strncmp(controllerMap, "8bitdopro2x", 63) != 0) {
                        handleFrequencyDown();
                    }
                }
            }

            if (secondaryControlAllowed && !primaryControlAllowed) {
                if (event.code == buttonO && event.value == 1) { // Triangle + O
                    handleAux();
                }

                if (event.code == buttonRB && event.value == 1) { // Triangle + RB
                    handleAuxOsc();
                }

                if (event.code == buttonSquare && event.value == 1) { // Triangle + Square
                    handleActiveControllerCycle();
                }
            }

            printf("\nKey %d pressed with value %d", event.code, event.value);
            break;
        case EV_ABS:
            if (strncmp(controllerMap, "8bitdopro2x", 63) == 0) {
                // keep in sync with above
                if (primaryControlAllowed && !secondaryControlAllowed) {
                    if (event.code == buttonRT && event.value == 1023) { // X + RT
                        handleFrequencyDown();
                    }
                }

                printf("\nKey %d pressed with value %d", event.code, event.value);
            }
            break;
        }
    }

    close(fd[0]);
    return NULL;
}

int main(int argc, char** argv)
{
    struct ff_effect effects[8];
    struct input_event play;
    controllerPath[0] = "/dev/input/invalid";
    int i = 0;

    for (i = 1; i < argc; ++i) {
        if (strncmp(argv[i], "--help", 63) == 0) {
            printf("Usage: %s /dev/input/eventXX\n", argv[0]);
            printf("\tUp to %d controller event paths can be supplied\n", maxControllers);
            printf("Options:\n");
            printf("\t--emulate: for testing without a controller\n");
            printf("\t--heartbeat: start in heartbeat mode\n");
            printf("Adjust primary controller mapping via the following:\n");
            printf("\t--dualshock\n");
            printf("\t--8bitdopro2s\n");
            printf("\t--8bitdopro2x\n");
            exit(1);
        } else if (strncmp(argv[i], "--emulate", 63) == 0) {
            emulate = 1;
        } else if (strncmp(argv[i], "--heartbeat", 63) == 0) {
            heartbeat = 1;
        } else if (strncmp(argv[i], "--dualshock", 63) == 0) {
            controllerMap = "dualshock";
        } else if (strncmp(argv[i], "--8bitdopro2s", 63) == 0) {
            controllerMap = "8bitdopro2s";
        } else if (strncmp(argv[i], "--8bitdopro2x", 63) == 0) {
            controllerMap = "8bitdopro2x";
        } else {
            if (numControllers <= (maxControllers - 1)) {
                controllerPath[numControllers] = argv[i];
                numControllers += 1;
            }
        }
    }

    if (emulate != 1) {
        for (i = 0; i < numControllers; ++i) {
            fd[i] = open(controllerPath[i], O_RDWR | O_CLOEXEC);
            if (fd[i] == -1) {
                perror("Failed to open controller");
                if (emulate != 1) {
                    exit(1);
                }
            }
            printf("Device %s opened\n", controllerPath[i]);

            int tmpEffectNum = 0;
            printf("Uploading effect #%d (strong rumble, strength 0xFFFF, length 50) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0;
            effects[tmpEffectNum].replay.length = 50;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 1;
            printf("Uploading effect #%d (strong & weak rumble, strength 0xFFFF, length 50) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0xFFFF;
            effects[tmpEffectNum].replay.length = 50;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 2;
            printf("Uploading effect #%d (strong rumble, strength 0xFFFF, length 100) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0;
            effects[tmpEffectNum].replay.length = 100;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 3;
            printf("Uploading effect #%d (strong & weak rumble, strength 0xFFFF, length 100) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0xFFFF;
            effects[tmpEffectNum].replay.length = 100;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 4;
            printf("Uploading effect #%d (strong rumble, strength 0xFFFF, length 150) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0;
            effects[tmpEffectNum].replay.length = 150;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 5;
            printf("Uploading effect #%d (strong & weak rumble, strength 0xFFFF, length 150) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0xFFFF;
            effects[tmpEffectNum].replay.length = 150;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 6;
            printf("Uploading effect #%d (weak rumble, strength 0xFFFF, length 50) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0xFFFF;
            effects[tmpEffectNum].replay.length = 50;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }

            tmpEffectNum = 7;
            printf("Uploading effect #%d (strong rumble, strength 0xFFFF, length 25) ...", tmpEffectNum);
            effects[tmpEffectNum].type = FF_RUMBLE;
            effects[tmpEffectNum].id = -1;
            effects[tmpEffectNum].u.rumble.strong_magnitude = 0xFFFF;
            effects[tmpEffectNum].u.rumble.weak_magnitude = 0;
            effects[tmpEffectNum].replay.length = 25;
            effects[tmpEffectNum].replay.delay = 0;
            fflush(stdout);
            if (ioctl(fd[i], EVIOCSFF, &effects[tmpEffectNum]) == -1) {
                perror("Failed to load effect");
            } else {
                printf("OK (id %d)\n", effects[tmpEffectNum].id);
            }
        }

        printf("Uploaded all effects");

        // Set a controller mapping
        if (strncmp(controllerMap, "dualshock", 63) == 0 || strncmp(controllerMap, "8bitdopro2s", 63) == 0) {
            buttonX = 304;
            buttonO = 305;
            buttonTriangle = 307;
            buttonSquare = 308;
            buttonSelect = 315;
            buttonRB = 311;
            buttonRT = 313;
            printf("\nSet controller map to DualShock 3 & 4 / 8BitDo Pro 2 (Switch)");
        } else if (strncmp(controllerMap, "8bitdopro2x", 63) == 0) {
            buttonX = 304;
            buttonO = 305;
            buttonTriangle = 307;
            buttonSquare = 306;
            buttonSelect = 311;
            buttonRB = 309;
            buttonRT = 5;
            printf("\nSet controller map to 8BitDo Pro 2 (X-Input)");
        }

        pthread_t thread = 0;
        if (pthread_create(&thread, NULL, read_input, NULL) != 0) {
            perror("pthread_create");
            exit(1);
        }
    }

    c = 0;
    if (heartbeat) {
        usleepAmt = 50000;
    } else {
        usleepAmt = 25000 * startInterval;
    }
    int effectCounter = 0;
    int effectMode = 0;
    int auxCount = 0;
    int auxModeAscend = true;

    do {
        if (paused && !emulate) {
            usleep(50000);
        } else {
            if (heartbeat) {
                // Delay like a heart
                if (effectCounter > 1) {
                    effectCounter = 0;
                }
                if (effectCounter == 1) {
                    effectMode = 4;
                    if (auxMode == 1) {
                        effectMode += 1;
                    }

                    if (c > 1000) {
                        usleep(25000);
                    } else if (c > 900) {
                        usleep(50000);
                    } else if (c > 800) {
                        usleep(75000);
                    } else if (c > 700) {
                        usleep(100000);
                    } else if (c > 600) {
                        usleep(125000);
                    } else if (c > 500) {
                        usleep(150000);
                    } else if (c > 400) {
                        usleep(200000);
                    } else if (c > 300) {
                        usleep(250000);
                    } else if (c > 200) {
                        usleep(350000);
                    } else if (c > 100) {
                        usleep(450000);
                    } else if (c > 0) {
                        usleep(550000);
                    }
                } else {
                    effectMode = 2;
                    if (auxMode == 1) {
                        effectMode += 1;
                    }
                    usleep(100000);
                }
                effectCounter++;
            } else {
                // Constant effect
                effectMode = 0;
                if (auxMode == 1) {
                    effectMode += 1;
                }
                if (auxMode >= 2) {
                    if (auxCount <= (3 * (9 - auxMode))) {
                        effectMode = 6;
                    } else if (auxCount >= (3 * (9 - auxMode) * 2)) {
                        effectMode = 7;
                        auxCount = 0;
                    } else {
                        effectMode = 7;
                    }
                    auxCount++;
                    if (auxModeOsc && c % (75 * auxMode) == 0) {
                        if (auxMode == 7) {
                            auxModeAscend = false;
                        }
                        if (auxMode == 2) {
                            auxModeAscend = true;
                        }
                        if (auxModeAscend && auxMode < 7) {
                            auxMode++;
                            printf("\nSpeeding up auxMode: %d", auxMode);
                        } else if (auxMode > 2) {
                            auxMode--;
                            printf("\nSlowing down auxMode: %d", auxMode);
                        }
                    }
                }
            }

            // Prepare the effect
            memset(&play, 0, sizeof(play));
            play.type = EV_FF;
            play.code = effects[effectMode].id;
            play.value = 1;

            // Play it
            if (emulate != 1) {
                for (i = 0; i < numControllers; ++i) {
                    if (activeControllers == 0 || activeControllers == (i + 1)) {
                        if (write(fd[i], (const void*)&play, sizeof(play)) == -1) {
                            perror("Failed to play effect");
                            exit(1);
                        }
                    }
                }
            }

            // Delay
            printf("\nCount: %d, Delay: %d, Effect: %d", c, usleepAmt, effectMode);
            if (heartbeat && (effectMode == 4 || effectMode == 5)) {
                usleep(usleepAmt * 4);
            } else {
                usleep(usleepAmt);
            }

            // Do an occasional longer delay
            if (heartbeat == 0) {
                if (delayMode == 1 && c % randomRange(5, 10) == 0) {
                    int periodSleep = usleepAmt * 4;
                    periodSleep = randomRange(periodSleep - 25000, periodSleep + 25000);
                    printf(", Random Period Delay: %d", periodSleep);
                    usleep(periodSleep);
                }
                if (delayMode > 1 && c % (delayMode - 1) == 0) {
                    int periodSleep = usleepAmt * 4;
                    periodSleep = randomRange(periodSleep - 25000, periodSleep + 25000);
                    printf(", Stable Period Delay: %d, Stable Delay Interval: %d", periodSleep, (delayMode - 1));
                    usleep(periodSleep);
                }
                if (c >= 300 && c % 2000 == 0) {
                    int periodSleep = randomRange(2000000, 30000000);
                    printf(", Extended Period Delay: %d", periodSleep);
                    // usleep(periodSleep);
                }
            }

            if (emulate && heartbeat) {
                if (c > 9 && c % 10 == 0) {
                    c += 90;
                }
            }

            c++;
        }
    } while (1);
    exit(0);
}

# Connected-signals study (simplified simulator, not SUMO)

Seeds per cell: 3. Duration 1800 s. Messages lost: 0%. Google weights: SYNTHETIC profile.

Delay is time spent waiting in queues per vehicle, including vehicles still inside when the run ends. Change is against isolated adaptive control.

## corridor

| scenario | variant | mean delay s | vs isolated | p95 s | blocked share | ped wait s | unfinished |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| light | fixed | 42 | +110% | 88 | 0.00 | 12 | 17 |
| light | isolated | 20 | +0% | 44 | 0.00 | 7 | 16 |
| light | connected | 20 | -1% | 43 | 0.00 | 7 | 17 |
| light | connected_ai | 20 | -2% | 42 | 0.00 | 7 | 17 |
| light | connected_gmaps | 20 | -1% | 43 | 0.00 | 7 | 17 |
| balanced | fixed | 46 | +80% | 90 | 0.00 | 13 | 30 |
| balanced | isolated | 26 | +0% | 51 | 0.00 | 8 | 24 |
| balanced | connected | 24 | -7% | 47 | 0.00 | 8 | 25 |
| balanced | connected_ai | 24 | -8% | 47 | 0.00 | 8 | 24 |
| balanced | connected_gmaps | 23 | -10% | 46 | 0.00 | 8 | 23 |
| peak | fixed | 273 | +41% | 631 | 0.20 | 15 | 600 |
| peak | isolated | 194 | +0% | 507 | 0.18 | 18 | 455 |
| peak | connected | 138 | -29% | 490 | 0.02 | 14 | 279 |
| peak | connected_ai | 133 | -31% | 441 | 0.02 | 14 | 263 |
| peak | connected_gmaps | 137 | -29% | 512 | 0.01 | 12 | 281 |
| surge | fixed | 417 | +4% | 1081 | 0.31 | 13 | 1208 |
| surge | isolated | 402 | +0% | 996 | 0.38 | 21 | 1163 |
| surge | connected | 280 | -30% | 829 | 0.19 | 22 | 849 |
| surge | connected_ai | 283 | -30% | 850 | 0.19 | 22 | 874 |
| surge | connected_gmaps | 272 | -32% | 835 | 0.14 | 21 | 827 |

## grid_mixed

| scenario | variant | mean delay s | vs isolated | p95 s | blocked share | ped wait s | unfinished |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| light | fixed | 31 | +111% | 65 | 0.00 | - | 13 |
| light | isolated | 15 | +0% | 35 | 0.00 | - | 8 |
| light | connected | 14 | -3% | 33 | 0.00 | - | 8 |
| light | connected_ai | 14 | -3% | 33 | 0.00 | - | 8 |
| light | connected_gmaps | 14 | -3% | 33 | 0.00 | - | 8 |
| balanced | fixed | 32 | +64% | 67 | 0.00 | - | 22 |
| balanced | isolated | 20 | +0% | 40 | 0.00 | - | 18 |
| balanced | connected | 19 | -2% | 40 | 0.00 | - | 18 |
| balanced | connected_ai | 19 | -4% | 39 | 0.00 | - | 17 |
| balanced | connected_gmaps | 19 | -3% | 40 | 0.00 | - | 17 |
| peak | fixed | 529 | +117% | 1322 | 0.52 | - | 1415 |
| peak | isolated | 244 | +0% | 724 | 0.29 | - | 812 |
| peak | connected | 108 | -56% | 471 | 0.00 | - | 202 |
| peak | connected_ai | 111 | -54% | 358 | 0.02 | - | 216 |
| peak | connected_gmaps | 102 | -58% | 503 | 0.00 | - | 200 |
| surge | fixed | 698 | +17% | 1515 | 0.66 | - | 2193 |
| surge | isolated | 596 | +0% | 1364 | 0.60 | - | 1997 |
| surge | connected | 344 | -42% | 996 | 0.27 | - | 1216 |
| surge | connected_ai | 350 | -41% | 947 | 0.27 | - | 1159 |
| surge | connected_gmaps | 285 | -52% | 912 | 0.17 | - | 1003 |

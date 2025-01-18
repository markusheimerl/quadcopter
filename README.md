# sim
A quadcoptor simulation

![Drone Simulation](https://raw.githubusercontent.com/markusheimerl/sim/1b0bb2c1bb29e7adcfb21ce206eb0af47999c9af/2025-01-14_09-25-44_flight.gif)


## Optional acceleration with Intel OneAPI

```bash
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | sudo tee /etc/apt/sources.list.d/oneAPI.list

sudo apt update
sudo apt install intel-oneapi-mkl intel-oneapi-mkl-devel
source /opt/intel/oneapi/setvars.sh
```
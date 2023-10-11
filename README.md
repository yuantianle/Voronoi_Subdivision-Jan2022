# Voronoi_Subdivision-Jan2022

## Section 1. Voronoi Type with different seed distribution

|Exponential|Grid|Irregular|Regular|
|-|-|-|-|
|<img width= 900px src="https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/a0cd0fce-8965-4056-91d4-9850eb84a2c9"></img>|<img width= 1100px src="https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/9ba63056-8064-4e4c-b161-69c6b718420b"></img>|![Voronoi_Map_Irregular](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/e2d35e2f-abcb-4a19-9042-0f7fe75fd94a)|![Voronoi_Map_Regular](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/6353d812-2fe7-4779-978a-54655dfbec53)

## Section 2. Methodology

|Original Voronoi|Assign Density|Redistributing seeds|Generate Sub Voronoi meshes|
|-|-|-|-|
|![Voronoi_Map_1st_level](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/9d66000a-3e7f-4f23-b2bf-9789e2a00395)|![Density_Map](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/2da75823-7023-47e8-bdb2-5c9836962140)|<img width= 820px src="https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/ac9ef014-b41c-4a35-af7c-6f06e6fd8e77"></img>|![Density_Voronoi](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/f683f9a3-ead5-4e4a-b0d8-95283d4fe295)|


|Father edges based on kids|Father edges based on itself|
|-|-|
|![Voronoi_Map_2nd_level_discritized_edge](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/8071f738-443c-41dd-86e3-942d7a2997cd)|![Voronoi_Map_2nd_level_straight_edge](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/4bb5b073-7951-4d38-99a5-d24984cf697f)|

## Section 3. Art Stylization based on the algorithm

### Some results

|![bone](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/84c62b3e-8a01-4b19-863b-12745cf4398c)|![m](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/f9263251-ce25-48c3-8483-b6ef1828e231)|![ppk](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/e2b9cfe2-9d37-4452-8cde-ac64b18e3363)|
|-|-|-|
|![53ffedebc0109990c0858364f143b66](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/d37791e7-b94e-4de0-8717-327ba508a519)|![612312bc57a049b97e0564092e7a8c2](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/d56a19ff-4951-4df8-90b3-e7b9338f97a6)|![f341e35f1cd1fe763ec8bf6ac2232f5](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/4ce4bc28-a206-4823-aa53-f0f337341347)|

### Different Voronoi seed density & Postprocessing

|Normal Result|Increasing Seed Density|Postprocessing
|-|-|-|
|![53ffedebc0109990c0858364f143b66](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/d37791e7-b94e-4de0-8717-327ba508a519)|![f8ff387b19d6d9a405aa90fb9d91190](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/0cb07f4c-0321-4fbf-8f06-5ace6b35b578)|![1312ab959c0716c47998c69a22822e9](https://github.com/yuantianle/Voronoi_Subdivision-Jan2022/assets/61530469/a02c331f-608f-40eb-af1f-ea31d38afb65)|

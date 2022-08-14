import pygrib

file = '../../../data/grib/gfsanl_3_20190403_0000_000.grb2'  # example filename

gr = pygrib.open(file)
for g in gr:
    # print(g)
    print(g.typeOfLevel, g.level, g.name, g.validDate, g.analDate, g.forecastTime)
    lats, lons = g.latlons()
    print(lats.shape, lats.min(), lats.max(), lons.shape, lons.min(), lons.max())

    print('-'*50)
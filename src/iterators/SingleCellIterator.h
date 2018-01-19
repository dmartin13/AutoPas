/*
 * SingleCellIterator.h
 *
 *  Created on: 17 Jan 2018
 *      Author: tchipevn
 */

#ifndef DEPENDENCIES_EXTERNAL_AUTOPAS_SRC_SINGLECELLITERATOR_H_
#define DEPENDENCIES_EXTERNAL_AUTOPAS_SRC_SINGLECELLITERATOR_H_

#include "cells/RMMParticleCell.h"

namespace autopas {

template<class Particle, class ParticleCell>
class SingleCellIterator {
public:
	SingleCellIterator() :_cell(nullptr), _index(0) {}
	SingleCellIterator(ParticleCell * cell_arg, int ind = 0) : _cell(cell_arg), _index(ind) {}
	Particle& operator *  () const {
		Particle * ptr = nullptr;
		_cell->moleculesAt(_index, ptr);
		return *ptr;
	}
	Particle* operator->() const {
		return &(this->operator*());
	}
	void operator ++() { ++_index; }
	bool isValid() { return _cell != nullptr and _index < _cell->numParticles(); }

	int getIndex() const { return _index; }

private:
	ParticleCell * _cell;
	int _index;
};

template<class Particle>
class SingleCellIterator<Particle, RMMParticleCell<Particle>>  {
public:
	SingleCellIterator() :_cell(nullptr), _index(0) {}
	SingleCellIterator(RMMParticleCell<Particle> * cell_arg, int ind = 0) : _cell(cell_arg), _index(ind) {}
	Particle& operator *() const {
		Particle * ptr = nullptr;
		ptr = const_cast<Particle *>(& _AoSReservoir);
		_cell->moleculesAt(_index, ptr);
		return *ptr;
	}
	Particle* operator->() const {
		return &(this->operator*());
	}
	void operator ++() { ++_index; }
	bool isValid() { return _cell != nullptr and _index < _cell->numParticles(); }

private:
	RMMParticleCell<Particle> * _cell;
	Particle _AoSReservoir;
	int _index;
};

} /* namespace autopas */

#endif /* DEPENDENCIES_EXTERNAL_AUTOPAS_SRC_SINGLECELLITERATOR_H_ */
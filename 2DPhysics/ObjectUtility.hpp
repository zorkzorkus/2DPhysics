#pragma once

#include "Circle.hpp"
#include "Box.hpp"

template<typename T>
inline T* IsObjectType(Object* o) = delete;

template<>
inline Circle* IsObjectType<Circle>(Object* o) {
	if (o->GetType() == Object::ObjectType::Circle) return reinterpret_cast<Circle*>(o);
	return nullptr;
}

template<>
inline Box* IsObjectType<Box>(Object* o) {
	if (o->GetType() == Object::ObjectType::Box) return reinterpret_cast<Box*>(o);
	return nullptr;
}